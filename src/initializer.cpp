#include "initializer.h"
#include <Eigen/Dense>
#include "eskf_utils.h"

namespace Localization
{

    Initializer::Initializer(const Eigen::Vector3d &init_I_p_Gps, double acc_std_limit, double g)
        : init_I_p_Gps_(init_I_p_Gps), acc_std_limit_(acc_std_limit), gravity_norm_(g) {}

    bool Initializer::Execute(const std::deque<GpsDataPtr> &gps_buffer, const std::deque<ImuDataPtr> &imu_buffer, State &state, Eigen::Vector3d &init_lla)
    {

        // 求均值
        Eigen::Vector3d lla_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d gps_cov = Eigen::Vector3d::Zero();

        for (const auto &gps_data : gps_buffer)
        {
            lla_mean += gps_data->lla;
            gps_cov += gps_data->cov.diagonal();
        }
        lla_mean /= gps_buffer.size();
        gps_cov /= gps_buffer.size();
        // 计算方差
        Eigen::Vector3d std2 = Eigen::Vector3d::Zero();
        for (const auto &gps_data : gps_buffer)
        {
            Eigen::Vector3d enu;
            ConvertLLAToENU(lla_mean, gps_data->lla, enu);
            std2 += enu.cwiseAbs2();
        }
        std2 /= gps_buffer.size();
        // 判断方差是否过大
        if (std2.x() > gps_cov.x() || std2.y() > gps_cov.y() || std2.z() > gps_cov.z())
        {
            printf("The std of the gps (%f,%f,%f) is too large!\n", std2.x(), std2.y(), std2.z());
            return false;
        }
        printf("init gps pos std is (%f,%f,%f)\n", std2.x(), std2.y(), std2.z());
        init_lla = lla_mean;

        const ImuDataPtr last_imu_ptr = imu_buffer.back();
        

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
        if (!ComputeG_R_IFromImuData(&state.G_R_I, imu_buffer))
        {
            printf("Failed to compute G_R_I from imu data!\n");
            return false;
        }
        // Set bias to zero.
        state.acc_bias.setZero();
        state.gyro_bias.setZero();
        // Set covariance.
        state.cov.setZero();
        state.cov.block<3, 3>(0, 0) = gps_cov.asDiagonal();
        state.cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero(); //静止初始化
        // roll pitch std 10 degree.
        state.cov.block<2, 2>(6, 6) = 10. * kDegreeToRadian * 10. * kDegreeToRadian * Eigen::Matrix2d::Identity();
        state.cov(8, 8) = 100. * kDegreeToRadian * 100. * kDegreeToRadian; // yaw std: 100 degree.
        // Acc bias.
        state.cov.block<3, 3>(9, 9) = 0.01 * Eigen::Matrix3d::Identity();
        // Gyro bias.
        state.cov.block<3, 3>(12, 12) = 0.01 * Eigen::Matrix3d::Identity();

        return true;
    }

    bool Initializer::Execute(const std::deque<ImuDataPtr> &imu_buffer, State &state)
    {

        const ImuDataPtr last_imu_ptr = imu_buffer.back();

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
        if (!ComputeG_R_IFromImuData(&state.G_R_I, imu_buffer))
        {
            printf("Failed to compute G_R_I from imu data!\n");
            return false;
        }
        // Set bias to zero.
        state.acc_bias.setZero();
        state.gyro_bias.setZero();
        // Set covariance.
        state.cov.setZero();
        return true;
    }

    bool Initializer::ComputeG_R_IFromImuData(Eigen::Matrix3d *G_R_I, const std::deque<ImuDataPtr> &imu_buffer)
    {
        // Compute mean and std of the imu buffer.
        Eigen::Vector3d sum_acc(0., 0., 0.);
        for (const auto imu_data : imu_buffer)
        {
            sum_acc += imu_data->acc;
        }
        const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer.size();

        Eigen::Vector3d sum_err2(0., 0., 0.);
        for (const auto &imu_data : imu_buffer)
        {
            sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2(); // cwiseAbs2() element-wise square
        }
        const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer.size()).cwiseSqrt();

        if (std_acc.maxCoeff() > acc_std_limit_)
        {
            printf("The std of the acceleration is too large, or the system is not static!\n");
            return false;
        }
        // imu坐标系为FLU，世界坐标系为ENU,静止时f = a - g = 0 - g = (0,0,1)
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