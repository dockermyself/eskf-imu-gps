#include "eskf.h"
#include "eskf_utils.h"

namespace Localization
{

    Eskf::Eskf(const double acc_noise, const double gyro_noise,
               const double acc_bias_noise, const double gyro_bias_noise,
               const Eigen::Vector3d &I_p_Gps, double g_norm)
        : initialized_(false)
    {
        initializer_ = std::make_unique<Initializer>(I_p_Gps, acc_noise * 2, g_norm);
        //重力为{0,0,-g},世界坐标系为ENU坐标系
        imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise,
                                                        acc_bias_noise, gyro_bias_noise,
                                                        Eigen::Vector3d(0., 0., -g_norm));
        gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);
    }

    bool Eskf::ProcessImuData(const ImuDataPtr imu_data_ptr)
    {
        if (!initialized_)
        {
            initializer_->AddImuData(imu_data_ptr);
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
            // Initialize the initial gps point used to convert lla to ENU.
            if (!initializer_->AddGpsData(gps_data_ptr, state_, init_lla_))
            {
                return false;
            }

            initialized_ = true;
            printf("System initialized!\n");
            return true;
        }

        // Update.
        gps_processor_->UpdateStateByGps(init_lla_, gps_data_ptr, state_);

        return true;
    }

} // namespace Localization