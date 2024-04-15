#include "eskf.h"
#include "eskf_utils.h"
#include <chrono>

namespace Localization
{
    Eskf::Eskf(const double acc_noise, const double gyro_noise,
               const double acc_bias_noise, const double gyro_bias_noise,
               const Eigen::Vector3d &I_p_Gps, double g_norm, double max_speed)
        : initialized_(false), max_speed_(max_speed)
    {
        initializer_ = std::make_unique<Initializer>(I_p_Gps, acc_noise * 2, g_norm);
        // 重力为{0,0,-g},世界坐标系为ENU坐标系
        imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise,
                                                        acc_bias_noise, gyro_bias_noise,
                                                        Eigen::Vector3d(0., 0., -g_norm));
        gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);
    }

    bool Eskf::ProcessImuData(const ImuDataPtr imu_data_ptr)
    {

        if (!initialized_)
        {
            AddImuData(imu_data_ptr);
            return false;
        }

        // Predict.
        imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);
        AddState(state_);
        return true;
    }

    bool Eskf::ImuOdometry(const ImuDataPtr imu_data_ptr)
    {
        if (!initialized_)
        {
            AddImuData(imu_data_ptr);
            if (ImuBufferFull() && initializer_->Execute(imu_buffer_, state_))
            {
                initialized_ = true;
                printf("System initialized!\n");
                return true;
            }
            return false;
        }

        // Predict.
        imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

        return true;
    }

    bool Eskf::ProcessGpsData(const GpsDataPtr gps_data_ptr)
    {
        if (!initialized_)
        {
            AddGpsData(gps_data_ptr);
            // Initialize the initial gps point used to convert lla to ENU.
            if (!GpsBufferFull() || !ImuBufferFull())
            {
                printf("Gps data or Imu data is not enough to initialize!\n");
                return false;
            }
            if (!initializer_->Execute(gps_data_buffer_, imu_buffer_, state_, init_lla_))
            {
                return false;
            }
            initialized_ = true;
            printf("System initialized!\n");
            return true;
        }

        // GPS相对于世界坐标系原点的位置
        Eigen::Vector3d gps_pos;
        // Convert wgs84 to ENU frame.
        ConvertLLAToENU(init_lla_, gps_data_ptr->lla, gps_pos);

        // GPS 定位时间
        double gps_timestamp = gps_data_ptr->timestamp;
        Eigen::Matrix3d gps_cov = gps_data_ptr->cov;

        State gps_state = FindClosestState(gps_timestamp);
        const double dt = state_.timestamp - gps_state.timestamp;
        double max_distance = max_speed_ * dt;
        Eigen::Vector3d imu_pos_move = state_.G_p_I - gps_state.G_p_I;

        if (imu_pos_move.norm() > max_distance)
        {
            Eigen::Matrix3d cov = Eigen::Vector3d(max_distance*max_distance, max_distance*max_distance, max_distance*max_distance).asDiagonal();
            gps_cov = gps_cov + cov;
        }
        else
        {
            gps_pos = gps_pos + imu_pos_move;
            gps_cov = gps_cov + state_.cov.block<3, 3>(0, 0) - gps_state.cov.block<3, 3>(0, 0);
        }

        gps_processor_->UpdateStateByGps(gps_pos, gps_cov, state_);
        AddGpsData(gps_data_ptr);
        return true;
    }

} // namespace Localization