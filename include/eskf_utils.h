#ifndef _ESFK_UTILS_H_
#define _ESFK_UTILS_H_

#include <memory>
#include <Eigen/Core>

namespace Localization
{
    struct ImuData
    {
        double timestamp;     // In second.
        Eigen::Vector3d acc;  // Acceleration in m/s^2
        Eigen::Vector3d gyro; // Angular velocity in radian/s.
        ImuData(double _timestamp, const Eigen::Vector3d &_acc, const Eigen::Vector3d &_gyro)
            : timestamp(_timestamp), acc(_acc), gyro(_gyro) {}
    };
    using ImuDataPtr = std::shared_ptr<ImuData>;

    struct GpsData
    {
        double timestamp;    // In second.
        Eigen::Vector3d lla; // Latitude in degree, longitude in degree, and altitude in meter.
        Eigen::Matrix3d cov; // Covariance in m^2.
        GpsData(double _timestamp, const Eigen::Vector3d &_lla, const Eigen::Matrix3d &_cov)
            : timestamp(_timestamp), lla(_lla), cov(_cov) {}
    };
    using GpsDataPtr = std::shared_ptr<GpsData>;

    struct State
    {
        double timestamp;
        Eigen::Vector3d G_p_I;     // The original point of the IMU frame in the Global frame.
        Eigen::Vector3d G_v_I;     // The velocity original point of the IMU frame in the Global frame.
        Eigen::Matrix3d G_R_I;     // The rotation from the IMU frame to the Global frame.
        Eigen::Vector3d acc_bias;  // The bias of the acceleration sensor.
        Eigen::Vector3d gyro_bias; // The bias of the gyroscope sensor.

        // Covariance.
        Eigen::Matrix<double, 15, 15> cov;

        // The imu data.
        ImuDataPtr imu_data_ptr;
    };

} // Localization

namespace Localization
{
    constexpr double kDegreeToRadian = M_PI / 180.;
    constexpr double kRadianToDegree = 180. / M_PI;
    constexpr double a = 6378137.0;
    constexpr double b = 6356752.3142;
    constexpr double kEarthRadius = (a + b) / 2.0;

    inline void ConvertLLAToENU(const Eigen::Vector3d &init_lla, const Eigen::Vector3d &point_lla, Eigen::Vector3d &enu)
    {
        double lat_rad = init_lla.x() * kDegreeToRadian;
        double lon_rad = init_lla.y() * kDegreeToRadian;

        double delta_lat = (point_lla.x() - init_lla.x()) * kDegreeToRadian;
        double delta_lon = (point_lla.y() - init_lla.y()) * kDegreeToRadian;

        enu.x() = delta_lon * cos(lat_rad) * kEarthRadius;
        enu.y() = delta_lat * kEarthRadius;
        enu.z() = point_lla.z() - init_lla.z();
    }


    inline Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d w;
        w << 0., -v(2), v(1), v(2), 0., -v(0), -v(1), v(0), 0.;
        return w;
    }

} // namespace Localization

#endif // _ESFK_UTILS_H_