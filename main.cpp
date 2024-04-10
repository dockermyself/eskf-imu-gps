#include <iostream>
#include <fstream>
#include <thread>
#include <eigen3/Eigen/Core>
#include "eskf.h"
#include "imu_reader.h"
#include "gps_reader.h"
#include "eskf_utils.h"

namespace Localization
{
    enum GPS_STATUS
    {
        GPS_STATUS_NO_FIX = 0,
        GPS_STATUS_FIX = 1,
    };

    /*
    #Accelerometer
    accelerometer_noise_density: 0.013766600200452252
    accelerometer_random_walk: 0.000467783974141409

    #Gyroscope
    gyroscope_noise_density: 0.0005230936266820257
    gyroscope_random_walk: 2.717496598492677e-06

    */
    // 动态噪声假定为静态标定结果的10倍
    const double kAccNoiseDensity = 0.013766600200452252 * 10;
    const double kGyroNoiseDensity = 0.0005230936266820257 * 10;
    const double kAccBiasRandomWalk = 0.000467783974141409 * 10;
    const double kGyroBiasRandomWalk = 2.717496598492677e-06 * 10;
    /*
    Misalignment Matrix
        1 -1.07293e-05   0.00109643
        0            1  -0.00677617
        0            0            1
    */
    const double kAccMisalignment[9] = {1, -1.07293e-05, 0.00109643, 0, 1, -0.00677617, 0, 0, 1};
    /*
    Scale Matrix
    1.00194     0       0
    0       1.00224     0
    0           0     1.004
    */
    const double kAccScaleFactor[3] = {1.00194, 1.00224, 1.004};

    /*
    Bias Vector
    -0.0284018
     0.0502699
     0.781119
    */

    const Eigen::Vector3d kAccBias = {-0.0284018, 0.0502699, 0.781119};

    /*
    Misalignment Matrix
        1           -0.0160567      -0.00108876
    0.00703196           1          0.00474895
    0.000234249     0.00142972           1
    */
    const double kGyroMisalignment[9] = {1, -0.0160567, -0.00108876, 0.00703196, 1, 0.00474895, 0.000234249, 0.00142972, 1};
    /*
    Scale Matrix
    1.38582     0          0
    0        1.37703       0
    0           0       1.4008
    */
    const double kGyroScaleFactor[3] = {1.38582, 1.37703, 1.4008};

    /*
    Bias Vector
    1.37549e-05
    -1.37592e-05
    3.1575e-05
    */
    const Eigen::Vector3d kGyroBias = {1.37549e-05, -1.37592e-05, 3.1575e-05};

    // GPS相对于IMU的位置(杆臂)
    const Eigen::Vector3d kArmPos = {0.0, -0.2, 0.0};
    // 当地重力加速度
    double kGravityNorm = 9.794;

    class EskfSystem
    {

    public:
        // AccelCorrection
        Eigen::Vector3d AccelCorrection(const Eigen::Vector3d &acc)
        {
            Eigen::Matrix3d T = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(kAccMisalignment);
            Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
            K(0, 0) = kAccScaleFactor[0];
            K(1, 1) = kAccScaleFactor[1];
            K(2, 2) = kAccScaleFactor[2];
            return T * K * (-kGravityNorm * acc - kAccBias);
        }
        // GyroCorrection
        Eigen::Vector3d GyroCorrection(const Eigen::Vector3d &gyro)
        {
            Eigen::Matrix3d T = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(kGyroMisalignment);
            Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
            K(0, 0) = kGyroScaleFactor[0];
            K(1, 1) = kGyroScaleFactor[1];
            K(2, 2) = kGyroScaleFactor[2];
            return T * K * (gyro - kGyroBias);
        }
        // 采集IMU数据
        void CollectImuData(ImuReader &imu_reader)
        {
            static float last_timestamp = 0.0f;
            const float imu_interval = 0.01f;
            std::ofstream fout = std::ofstream("imu_data.bag");
            ImuOriginData data;

            auto start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start < std::chrono::hours(1))
            {
                if (imu_reader.getData(data))
                {
                    double timestamp = data.timestamp.value;

                    if (timestamp - last_timestamp > imu_interval)
                    {
                        Eigen::Vector3d acc = {data.accel[0].value, data.accel[1].value, data.accel[2].value};
                        acc = -kGravityNorm * acc;
                        printf("acc norm %f\n", acc.norm());
                        Eigen::Vector3d gyro = {data.gyro[0].value, data.gyro[1].value, data.gyro[2].value};
                        printf("timestamp: %.3f, acc: %.3f %.3f %.3f, gyro: %.3f %.3f %.3f\n",
                               timestamp, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
                        fout << timestamp << " ";
                        fout << gyro[0] << " " << gyro[1] << " " << gyro[2] << " ";
                        fout << acc[0] << " " << acc[1] << " " << acc[2] << " ";
                        fout << std::endl;
                        last_timestamp = timestamp;
                    }
                }
            }
        }

        void PrintImuData(ImuReader &imu_reader)
        {
            ImuOriginData data;
            static float last_timestamp = 0.0f;
            const float imu_interval = 0.1f;
            while (true)
            {
                if (imu_reader.getData(data))
                {
                    if (data.timestamp.value - last_timestamp > imu_interval)
                    {
                        Eigen::Vector3d acc = AccelCorrection({data.accel[0].value, data.accel[1].value, data.accel[2].value});
                        printf("acc norm %f\n", acc.norm());
                        Eigen::Vector3d gyro = GyroCorrection({data.gyro[0].value, data.gyro[1].value, data.gyro[2].value});
                        printf("timestamp: %.3f, acc: %.3f %.3f %.3f, gyro: %.3f %.3f %.3f\n",
                               data.timestamp.value, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
                        last_timestamp = data.timestamp.value;
                    }
                }
            }
        }
        // 采集GPS数据
        void CollectGpsData(GPSReader &gps_reader)
        {
            GpsOriginData data;
            std::ofstream fout = std::ofstream("gps_data.bag");
            while (true)
            {
                if (gps_reader.getData(data))
                {
                    printf("timestamp: %.3f, lat: %.9f, lon: %.9f, alt: %.3f\n",
                           data.timestamp, data.lat, data.lon, data.alt);
                    fout << data.timestamp << " ";
                    fout << data.lat << " " << data.lon << " " << data.alt << " ";
                    fout << std::endl;
                }
            }
        }

        void PrintGpsData(GPSReader &gps_reader)
        {
            GpsOriginData data;
            const int collect_times = 50;
            std::deque<GpsOriginData> gps_data;
            const float gps_error_th = 2.5;
            bool initial_status = false;
            while (true)
            {
                if (gps_reader.getData(data))
                {
                    if (initial_status)
                    {
                        Eigen::Vector3d enu;
                        ConvertLLAToENU({gps_data.front().lat, gps_data.front().lon, gps_data.front().alt},
                                        {data.lat, data.lon, data.alt}, enu);
                        printf("[gps pos] (%f,%f,%f)\n", enu.x(), enu.y(), enu.z());
                    }
                    else
                    {
                        gps_data.push_back(data);
                        if (gps_data.size() < collect_times + 1)
                        {
                            printf("gps buffer size %ld\n", gps_data.size());
                        }
                        else
                        {
                            initial_status = true;
                            for (int i = 0; i < collect_times + 1; i++)
                            {
                                Eigen::Vector3d enu;
                                // lla to enu
                                ConvertLLAToENU({gps_data.front().lat, gps_data.front().lon, gps_data.front().alt},
                                                {gps_data[i].lat, gps_data[i].lon, gps_data[i].alt}, enu);
                                if (enu.cwiseAbs().maxCoeff() > gps_error_th)
                                {
                                    initial_status = false;
                                    printf("gps init failed,pos = (%f,%f,%f)\n", enu.x(), enu.y(), enu.z());
                                    gps_data.pop_front();
                                    break;
                                }
                            }
                            if (initial_status)
                            {
                                printf("gps init success\n");
                            }
                        }
                    }
                }
            }
        }

        // 实现IMU数据读取
        const ImuDataPtr ReadImuData(ImuReader &imu_reader)
        {
            ImuOriginData data;
            if (imu_reader.getData(data))
            {
                Eigen::Vector3d acc = AccelCorrection({data.accel[0].value, data.accel[1].value, data.accel[2].value});
                Eigen::Vector3d gyro = GyroCorrection({data.gyro[0].value, data.gyro[1].value, data.gyro[2].value});
                return std::make_shared<Localization::ImuData>(data.timestamp.value, acc, gyro);
            }

            return nullptr;
        }

        const GpsDataPtr ReadGpsData(GPSReader &gps_reader)
        {
            GpsOriginData data;
            if (gps_reader.getData(data))
            {
                // printf("timestamp: %.3f, lat: %.9f, lon: %.9f, alt: %.3f\n",
                //        data.timestamp, data.lat, data.lon, data.alt);
                double x_noise = 1.5;
                double y_noise = 1.5;
                double z_noise = 1.5;
                if (GPS_STATUS_NO_FIX == data.status)
                {
                    double x_noise = 2.5;
                    double y_noise = 2.5;
                    double z_noise = 1.5;
                }
                // 计算gps噪声协方差
                Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
                cov(0, 0) = x_noise * x_noise;
                cov(1, 1) = y_noise * y_noise;
                cov(2, 2) = z_noise * z_noise;
                return std::make_shared<GpsData>(data.timestamp, Eigen::Vector3d(data.lat, data.lon, data.alt), cov);
            }
            return nullptr;
        }

        void Run(ImuReader &imu_reader, GPSReader &gps_reader)
        {
            std::unique_ptr<Localization::Eskf> localizer_ptr_ =
                std::make_unique<Localization::Eskf>(kAccNoiseDensity, kGyroNoiseDensity,
                                                     kAccBiasRandomWalk, kGyroBiasRandomWalk,
                                                     kArmPos, kGravityNorm);
            Localization::State current_state = {0};
            std::fstream eskf_state_file = std::fstream("eskf_state.bag", std::ios::out);
            std::fstream gps_pos_file = std::fstream("gps_pos.bag", std::ios::out);
            // 当前时间
            auto start = std::chrono::steady_clock::now();
            auto save_time = start;
            auto run_duration = std::chrono::seconds(20 * 60);
            auto save_interval = std::chrono::milliseconds(100);

            while (std::chrono::steady_clock::now() - start < run_duration)
            {
                const Localization::ImuDataPtr imu_data_ptr = ReadImuData(imu_reader);
                if (imu_data_ptr && localizer_ptr_->ProcessImuData(imu_data_ptr))
                {
                    if (std::chrono::steady_clock::now() - save_time > save_interval)
                    {
                        save_time = std::chrono::steady_clock::now();
                        current_state = localizer_ptr_->state();
                        // printf("[eskf state pos] (%f,%f,%f)\n", current_state.G_p_I.x(), current_state.G_p_I.y(), current_state.G_p_I.z());
                        // printf("[eskf state vel] (%f,%f,%f)\n", current_state.G_v_I.x(), current_state.G_v_I.y(), current_state.G_v_I.z());

                        Eigen::Quaterniond q(current_state.G_R_I);
                        q.normalize();
                        // printf("[eskf state q] (%f,%f,%f,%f)\n", q.x(), q.y(), q.z(), q.w());
                        Eigen::Vector3d acc_bias = current_state.acc_bias;
                        Eigen::Vector3d gyro_bias = current_state.gyro_bias;
                        // printf("[eskf state acc_bias] (%f,%f,%f)\n", acc_bias.x(), acc_bias.y(), acc_bias.z());
                        // printf("[eskf state gyro_bias] (%f,%f,%f)\n", gyro_bias.x(), gyro_bias.y(), gyro_bias.z());
                        eskf_state_file << current_state.timestamp << " ";
                        eskf_state_file << current_state.G_p_I.x() << " " << current_state.G_p_I.y() << " " << current_state.G_p_I.z() << " ";
                        eskf_state_file << current_state.G_v_I.x() << " " << current_state.G_v_I.y() << " " << current_state.G_v_I.z() << " ";
                        eskf_state_file << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
                        eskf_state_file << acc_bias.x() << " " << acc_bias.y() << " " << acc_bias.z() << " ";
                        eskf_state_file << gyro_bias.x() << " " << gyro_bias.y() << " " << gyro_bias.z() << " ";
                        eskf_state_file << std::endl;
                        // printf("imu predict state (%f,%f,%f)\n", current_state.G_p_I.x(), current_state.G_p_I.y(), current_state.G_p_I.z());
                    }
                }

                const Localization::GpsDataPtr gps_data_ptr = ReadGpsData(gps_reader);
                if (gps_data_ptr && localizer_ptr_->ProcessGpsData(gps_data_ptr))
                {
                    current_state = localizer_ptr_->state();
                    Eigen::Vector3d enu;
                    ConvertLLAToENU(localizer_ptr_->initial_lla(), gps_data_ptr->lla, enu);
                    // printf("[gps pos] (%f,%f,%f)\n", enu.x(), enu.y(), enu.z());
                    // 记录数据
                    gps_pos_file << gps_data_ptr->timestamp << " ";
                    gps_pos_file << enu.x() << " " << enu.y() << " " << enu.z() << " ";
                    gps_pos_file << std::endl;
                }
            }
            eskf_state_file.close();
            gps_pos_file.close();
        }
    };
}
int main(int argc, char **argv)
{
    // udevadm info -a -n /dev/ttyUSB1 | grep KERNELS    绑定USB口位置
    // udevadm info --attribute-walk --name=/dev/ttyUSB0 修改ttyUSB別名

    ImuReader imu_reader("/dev/imu", B921600);
    imu_reader.EnableReadThread();
    GPSReader gps_reader("/dev/gps", B9600);
    gps_reader.EnableReadThread();
    Localization::EskfSystem eskf_system;
    eskf_system.Run(imu_reader, gps_reader);
    // eskf_system.PrintImuData(imu_reader);

    return 0;
}
