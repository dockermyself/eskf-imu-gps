#include "initializer.h"
#include <Eigen/Dense>
#include "eskf_utils.h"

namespace Localization
{

    Initializer::Initializer(const Eigen::Vector3d &init_I_p_Gps, double acc_std_limit, double g)
        : init_I_p_Gps_(init_I_p_Gps), acc_std_limit_(acc_std_limit), gravity_norm_(g) {}

    void Initializer::AddImuData(const ImuDataPtr imu_data_ptr)
    {
        imu_buffer_.push_back(imu_data_ptr);

        if (imu_buffer_.size() > imu_buffer_size_)
        {
            imu_buffer_.pop_front();
        }
    }

    bool Initializer::AddGpsData(const GpsDataPtr gps_data_ptr, State &state, Eigen::Vector3d &init_lla)
    {
        gps_data_buffer_.push_back(gps_data_ptr);

        if (gps_data_buffer_.size() < gps_buffer_size_)
        {
            printf("Not enough gps data to initialize the system!\n");
            return false;
        }
        else
        {
            // 求均值
            Eigen::Vector3d lla_mean = Eigen::Vector3d::Zero();

            for (const auto &gps_data : gps_data_buffer_)
            {
                lla_mean += gps_data->lla;
            }
            lla_mean /= gps_buffer_size_;
            // 计算方差
            Eigen::Vector3d std2 = Eigen::Vector3d::Zero();
            for (const auto &gps_data : gps_data_buffer_)
            {
                Eigen::Vector3d enu;
                ConvertLLAToENU(lla_mean, gps_data->lla, enu);
                std2 += enu.cwiseAbs2();
            }
            std2 /= gps_buffer_size_;
            // 判断方差是否过大
            if (std2.x() > gps_data_ptr->cov(0, 0) || std2.y() > gps_data_ptr->cov(1, 1) || std2.z() > gps_data_ptr->cov(2, 2))
            {
                printf("The std of the gps (%f,%f,%f) is too large!\n", std2.x(), std2.y(), std2.z());
                gps_data_buffer_.pop_front();
                return false;
            }
            printf("init gps pos std is (%f,%f,%f)\n", std2.x(), std2.y(), std2.z());
            init_lla = lla_mean;
            gps_data_buffer_.pop_front();
        }

        if (imu_buffer_.size() < imu_buffer_size_)
        {
            printf("Not enough imu data to initialize the system!\n");
            return false;
        }

        const ImuDataPtr last_imu_ptr = imu_buffer_.back();
        // synchronize all sensors.
        if (std::abs(gps_data_ptr->timestamp - last_imu_ptr->timestamp) > 0.5)
        {
            printf("Gps data and imu data are not synchronized!\n");
            return false;
        }

        // Set timestamp and imu date.
        state.timestamp = last_imu_ptr->timestamp;
        state.imu_data_ptr = last_imu_ptr;

        // Set initial mean.
        state.G_p_I.setZero();

        // We have no information to set initial velocity.
        // So, just set it to zero and given big covariance.
        state.G_v_I.setZero();

        // We can use the direction of gravity to set roll and pitch.
        // But, we cannot set the yaw.
        // So, we set yaw to zero and give it a big covariance.
        if (!ComputeG_R_IFromImuData(&state.G_R_I))
        {
            printf("Failed to compute G_R_I from imu data!\n");
            return false;
        }
        // Set bias to zero.
        state.acc_bias.setZero();
        state.gyro_bias.setZero();
        // Set covariance.
        state.cov.setZero();
        state.cov.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity(); // position std: 10 m
        state.cov.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity(); // velocity std: 10 m/s
        // roll pitch std 10 degree.
        state.cov.block<2, 2>(6, 6) = 10. * kDegreeToRadian * 10. * kDegreeToRadian * Eigen::Matrix2d::Identity();
        state.cov(8, 8) = 100. * kDegreeToRadian * 100. * kDegreeToRadian; // yaw std: 100 degree.
        // Acc bias.
        state.cov.block<3, 3>(9, 9) = 0.1 * Eigen::Matrix3d::Identity();
        // Gyro bias.
        state.cov.block<3, 3>(12, 12) = 0.1 * Eigen::Matrix3d::Identity();

        return true;
    }

    bool Initializer::ComputeG_R_IFromImuData(Eigen::Matrix3d *G_R_I)
    {
        // Compute mean and std of the imu buffer.
        Eigen::Vector3d sum_acc(0., 0., 0.);
        for (const auto imu_data : imu_buffer_)
        {
            sum_acc += imu_data->acc;
        }
        const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

        Eigen::Vector3d sum_err2(0., 0., 0.);
        for (const auto &imu_data : imu_buffer_)
        {
            sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2(); // cwiseAbs2() element-wise square
        }
        const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

        if (std_acc.maxCoeff() > acc_std_limit_)
        {
            printf("The std of the acceleration is too large, or the system is not static!\n");
            return false;
        }
        //imu坐标系为FLU，世界坐标系为ENU,静止时f = a - g = 0 - g = (0,0,1)
        Eigen::Vector3d acc_norm = mean_acc.normalized();
        double roll = std::atan2(acc_norm.y(), acc_norm.z());
        double pitch = std::asin(-acc_norm.x());
        printf("roll: %f, pitch: %f\n", roll * kRadianToDegree, pitch * kRadianToDegree);
    
        *G_R_I = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::Vector3d acc = (*G_R_I).transpose() * Eigen::Vector3d(0., 0., 1);
        // check acc1 == acc2
        printf("acc1: %f, %f, %f\n", acc.x(), acc.y(), acc.z());
        printf("acc2: %f, %f, %f\n", acc_norm.x(), acc_norm.y(), acc_norm.z());
        return true;
    }

} // namespace Localization