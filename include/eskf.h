#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <Eigen/Core>
#include <vector>
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
             const Eigen::Vector3d &I_p_Gps, double g_norm ,double max_speed);

        bool ProcessImuData(const ImuDataPtr imu_data_ptr);

        bool ProcessGpsData(const GpsDataPtr gps_data_ptr);

        bool ImuOdometry(const ImuDataPtr imu_data_ptr);

        State state() const { return state_; }

        Eigen::Vector3d initial_lla() const { return init_lla_; }

    private:
        bool ImuBufferFull() const { return imu_buffer_.size() >= imu_buffer_size_; }

        bool GpsBufferFull() const { return gps_data_buffer_.size() >= gps_buffer_size_; }

        void AddGpsData(const GpsDataPtr &gps_data_ptr)
        {
            gps_data_buffer_.push_back(gps_data_ptr);
            if (gps_data_buffer_.size() > gps_buffer_size_)
            {
                gps_data_buffer_.pop_front();
            }
        }

        void AddImuData(const ImuDataPtr &imu_data_ptr)
        {
            imu_buffer_.push_back(imu_data_ptr);
            if (imu_buffer_.size() > imu_buffer_size_)
            {
                imu_buffer_.pop_front();
            }
        }

        void AddState(const State &state)
        {
            state_buffer_.push_back(state);
            if (state_buffer_.size() > state_buffer_size_)
            {
                state_buffer_.pop_front();
            }
        }

        // 二分法查找时间大于timestamp的最小状态
        State FindClosestState(double timestamp, int start, int end)
        {
            if (start == end)
            {
                return state_buffer_[start];
            }

            int mid = (start + end) / 2;
            if (state_buffer_[mid].timestamp < timestamp)
            {
                return FindClosestState(timestamp, mid + 1, end);
            }
            else
            {
                return FindClosestState(timestamp, start, mid);
            }
        }

        State FindClosestState(double timestamp)
        {
            return FindClosestState(timestamp, 0, state_buffer_.size() - 1);
        }

    private:
        bool initialized_;
        Eigen::Vector3d init_lla_; // The initial reference gps point.
        State state_;
        double max_speed_;
        std::unique_ptr<Initializer> initializer_;
        std::unique_ptr<ImuProcessor> imu_processor_;
        std::unique_ptr<GpsProcessor> gps_processor_;

        const int imu_buffer_size_ = 100; // 初始化时imu数据的缓存大小，用于获取acc均值和方差
        std::deque<ImuDataPtr> imu_buffer_;
        const int gps_buffer_size_ = 25;
        std::deque<GpsDataPtr> gps_data_buffer_;
        const int state_buffer_size_ = 100;
        std::deque<State> state_buffer_;
    };

} // namespace Localization

#endif // LOCALIZER_H