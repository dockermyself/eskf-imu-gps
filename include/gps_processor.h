#ifndef GPS_PROCESSOR_H_
#define GPS_PROCESSOR_H_

#include <Eigen/Dense>
#include <deque>
#include "eskf_utils.h"

namespace Localization
{
    class GpsProcessor
    {

    public:
        GpsProcessor(const Eigen::Vector3d &I_p_Gps);

        bool UpdateStateByGps(const Eigen::Vector3d &init_lla, const GpsDataPtr gps_data_ptr, State& state);

    private:
        void ComputeJacobianAndResidual(const Eigen::Vector3d &init_lla,
                                        const GpsDataPtr gps_data,
                                        const State &state,
                                        Eigen::Matrix<double, 3, 15> *jacobian,
                                        Eigen::Vector3d *residual);

        const Eigen::Vector3d I_p_Gps_;
    };

    void AddDeltaToState(const Eigen::Matrix<double, 15, 1> &delta_x, State& state);

} // namespace Localization

#endif // GPS_PROCESSOR_H_