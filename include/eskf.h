#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <Eigen/Core>
#include "eskf_utils.h"
#include "gps_processor.h"
#include "imu_processor.h"
#include "initializer.h"

namespace Localization
{
    class Eskf
    {
    public:
        Eskf(const double acc_noise, const double gyro_noise,
             const double acc_bias_noise, const double gyro_bias_noise,
             const Eigen::Vector3d &I_p_Gps, double g_norm = 9.8);

        bool ProcessImuData(const ImuDataPtr imu_data_ptr);

        bool ProcessGpsData(const GpsDataPtr gps_data_ptr);

        State state() const { return state_; }

        Eigen::Vector3d initial_lla() const { return init_lla_; }

    private:
        std::unique_ptr<Initializer> initializer_;
        std::unique_ptr<ImuProcessor> imu_processor_;
        std::unique_ptr<GpsProcessor> gps_processor_;

        bool initialized_;
        Eigen::Vector3d init_lla_; // The initial reference gps point.
        State state_;
    };

} // namespace Localization

#endif // LOCALIZER_H