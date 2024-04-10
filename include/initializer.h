#ifndef LOCALIZATION_INITIALIZER_H_
#define LOCALIZATION_INITIALIZER_H_

#include <deque>

#include "eskf_utils.h"

namespace Localization
{
    class Initializer
    {
        const int imu_buffer_size_ = 100; // 初始化时imu数据的缓存大小，用于获取acc均值和方差
        std::deque<ImuDataPtr> imu_buffer_;
        const int gps_buffer_size_ = 25;
        std::deque<GpsDataPtr> gps_data_buffer_;

        Eigen::Vector3d init_I_p_Gps_;
        const double acc_std_limit_;
        const double gravity_norm_;

    public:
        Initializer(const Eigen::Vector3d &init_I_p_Gps, double acc_std_limit, double g);

        void AddImuData(const ImuDataPtr imu_data_ptr);

        bool AddGpsData(const GpsDataPtr gps_data_ptr, State &state, Eigen::Vector3d &init_lla);


    private:
        bool ComputeG_R_IFromImuData(Eigen::Matrix3d *G_R_I);
    };

} // namespace Localization

#endif // LOCALIZATION_INITIALIZER_H_