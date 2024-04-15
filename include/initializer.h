#ifndef LOCALIZATION_INITIALIZER_H_
#define LOCALIZATION_INITIALIZER_H_

#include <deque>

#include "eskf_utils.h"

namespace Localization
{
    class Initializer
    {

        Eigen::Vector3d init_I_p_Gps_;
        const double acc_std_limit_;
        const double gravity_norm_;

    public:
        Initializer(const Eigen::Vector3d &init_I_p_Gps, double acc_std_limit, double g);

        bool Execute(const std::deque<GpsDataPtr> &gps_buffer, const std::deque<ImuDataPtr> &imu_buffer, State &state, Eigen::Vector3d &init_lla);
        bool Execute(const std::deque<ImuDataPtr> &imu_buffer, State &state);

    private:
        bool ComputeG_R_IFromImuData(Eigen::Matrix3d *G_R_I, const std::deque<ImuDataPtr> &imu_buffer);
    };

} // namespace Localization

#endif // LOCALIZATION_INITIALIZER_H_