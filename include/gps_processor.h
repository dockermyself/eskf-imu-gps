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

        bool UpdateStateByGps(const Eigen::Vector3d &gps_pos,const Eigen::Matrix3d& gps_cov, State &state);

    private:
        void ComputeJacobianAndResidual(const Eigen::Vector3d &gps_pos,
                                        const State &state,
                                        Eigen::Matrix<double, 3, 15> *jacobian,
                                        Eigen::Vector3d *residual);

        const Eigen::Vector3d I_p_Gps_;
    };

    void AddDeltaToState(const Eigen::Matrix<double, 15, 1> &delta_x, State &state);

} // namespace Localization

#endif // GPS_PROCESSOR_H_