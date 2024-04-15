#include <iostream>
#include <fstream>
#include <thread>
#include <eigen3/Eigen/Core>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
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
    double kIMUInterval = 0.01;
    /*
    X Velocity Random Walk: 0.00345 m/s/sqrt(s) 0.20692 m/s/sqrt(hr)
    Y Velocity Random Walk: 0.00309 m/s/sqrt(s) 0.18516 m/s/sqrt(hr)
    Z Velocity Random Walk: 0.00423 m/s/sqrt(s) 0.25356 m/s/sqrt(hr)
    X Bias Instability: 0.00022 m/s^2 2885.66064 m/hr^2
    Y Bias Instability: 0.00018 m/s^2 2316.19565 m/hr^2
    Z Bias Instability: 0.00060 m/s^2 7814.50546 m/hr^2
    X Accel Random Walk: 0.00004 m/s^2/sqrt(s)
    Y Accel Random Walk: 0.00003 m/s^2/sqrt(s)
    Z Accel Random Walk: 0.00014 m/s^2/sqrt(s)

    */
    // 实际噪声在标定结果的基础上乘以n倍
    const double kAccNoise = (0.00345 + 0.00309 + 0.00423) / 3 / sqrt(kIMUInterval);
    const double kAccBiasRandomWalk = (0.00022 + 0.00018 + 0.00060) / 3;
    /*
    X Angle Random Walk: 0.01061 deg/sqrt(s) 0.63638 deg/sqrt(hr)
    Y Angle Random Walk: 0.01106 deg/sqrt(s) 0.66349 deg/sqrt(hr)
    Z Angle Random Walk: 0.00975 deg/sqrt(s) 0.58473 deg/sqrt(hr)
    X Bias Instability: 0.00011 deg/s 0.39469 deg/hr
    Y Bias Instability: 0.00029 deg/s 1.04721 deg/hr
    Z Bias Instability: 0.00009 deg/s 0.30922 deg/hr
    X Rate Random Walk: 0.00005 deg/s/sqrt(s)
    Y Rate Random Walk: 0.00006 deg/s/sqrt(s)
    Z Rate Random Walk: 0.00005 deg/s/sqrt(s)
    */
    const double kGyroNoise = (0.01061 + 0.01106 + 0.00975) / 3 / sqrt(kIMUInterval) * kDegreeToRadian;
    const double kGyroBiasRandomWalk = (0.00011 + 0.00029 + 0.00009) / 3 * kDegreeToRadian;
    /*
    Misalignment Matrix
          1 0.000211026 -0.00592877
          0           1  -0.0075711
          0           0           1
    */
    const double kAccMisalignment[9] = {1, 0.000211026, -0.00592877, 0, 1, -0.0075711, 0, 0, 1};
    /*
    Scale Matrix
    1.01257       0       0
        0 1.00275       0
        0       0 1.00584
    */
    const double kAccScaleFactor[3] = {1.01257, 1.00275, 1.00584};

    /*
    Bias Vector
    -0.0933937
    0.0315169
    0.793868
    */

    const Eigen::Vector3d kAccBias = {-0.0933937, 0.0315169, 0.793868};

    /*
    Misalignment Matrix
        1       -0.00524587     -0.00778415
    -0.0187879           1      0.0119236
    -0.00803335  -0.0471048           1
    */
    const double kGyroMisalignment[9] = {1, -0.00524587, -0.00778415, -0.0187879, 1, 0.0119236, -0.00803335, -0.0471048, 1};
    /*
    Scale Matrix
    1.0104       0       0
        0  1.0126       0
        0       0 1.03008
    */
    const double kGyroScaleFactor[3] = {1.0104, 1.0126, 1.03008};

    /*
    Bias Vector
    1.99535e-05
    -0.000118125
    4.12334e-05
    */
    const Eigen::Vector3d kGyroBias = {1.99535e-05, -0.000118125, 4.12334e-05};

    // GPS相对于IMU的位置(杆臂)
    const Eigen::Vector3d kArmPos = {0.05, -0.1, 0.0};
    // 当地重力加速度
    double kGravityNorm = 9.794;

    double kVelocityMax = 2.0;

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
            const float imu_interval = 0.0f;
            std::ofstream fout = std::ofstream("imu_data.bag");
            ImuOriginData data;

            auto start = std::chrono::steady_clock::now();
            auto duration = std::chrono::seconds(20 * 60);
            while (std::chrono::steady_clock::now() - start < duration)
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

        void LoggerWrite(const char *data, int len, const char *remote, int port)
        {
            static int logger_fd = -1;
            if (logger_fd < 0)
            {
                logger_fd = socket(AF_INET, SOCK_DGRAM, 0);
                if (logger_fd < 0)
                {
                    printf("socket create error\n");
                    return;
                }
                struct sockaddr_in server_addr;
                memset(&server_addr, 0, sizeof(server_addr));
                server_addr.sin_family = AF_INET;
                server_addr.sin_port = htons(port);
                server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

                if (bind(logger_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                {
                    printf("bind error\n");
                    return;
                }
            }

            struct sockaddr_in client_addr;
            memset(&client_addr, 0, sizeof(client_addr));
            client_addr.sin_family = AF_INET;
            client_addr.sin_port = htons(port);
            client_addr.sin_addr.s_addr = inet_addr(remote);
            sendto(logger_fd, data, len, 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
        }
        // 绘制(x,y)轨迹
        void LoggerWriteTrajectory(double x, double y, const char *remote, int port)
        {
            char str_buffer[100] = {0};
            sprintf(str_buffer, "pos %.3f %.3f", x, y);
            LoggerWrite(str_buffer, strlen(str_buffer), remote, port);
        }

        // 绘制 GPS 轨迹
        void LoggerWriteGps(double x, double y, const char *remote, int port)
        {
            char str_buffer[100] = {0};
            sprintf(str_buffer, "gps %.3f %.3f", x, y);
            LoggerWrite(str_buffer, strlen(str_buffer), remote, port);
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
        // Imu 里程计
        void ImuOdometry(ImuReader &imu_reader)
        {
            std::unique_ptr<Localization::Eskf> localizer_ptr_ =
                std::make_unique<Localization::Eskf>(kAccNoise, kGyroNoise,
                                                     kAccBiasRandomWalk, kGyroBiasRandomWalk,
                                                     Eigen::Vector3d(0, 0, 0), kGravityNorm, kVelocityMax);
            Localization::State current_state;
            std::fstream imu_odometry_file = std::fstream("imu_odometry_state.bag", std::ios::out);
            std::fstream gps_pos_file = std::fstream("gps_pos.bag", std::ios::out);
            // 当前时间
            auto start = std::chrono::steady_clock::now();
            auto save_time = start;
            auto run_duration = std::chrono::seconds(60 * 100);
            auto save_interval = std::chrono::milliseconds(100);

            while (std::chrono::steady_clock::now() - start < run_duration)
            {
                const Localization::ImuDataPtr imu_data_ptr = ReadImuData(imu_reader);
                if (imu_data_ptr && localizer_ptr_->ImuOdometry(imu_data_ptr))
                {

                    if (std::chrono::steady_clock::now() - save_time > save_interval)
                    {
                        save_time = std::chrono::steady_clock::now();
                        current_state = localizer_ptr_->state();
                        printf("[imu predict state timestamp %f\n", current_state.timestamp);
                        printf("[imu predict state cov] (%f,%f,%f)\n", current_state.cov.diagonal().x(), current_state.cov.diagonal().y(), current_state.cov.diagonal().z());
                        printf("[imu predict state pos] (%f,%f,%f)\n", current_state.G_p_I.x(), current_state.G_p_I.y(), current_state.G_p_I.z());
                        printf("[imu predict state vel] (%f,%f,%f)\n", current_state.G_v_I.x(), current_state.G_v_I.y(), current_state.G_v_I.z());
                        // LoggerWriteTrajectory(current_state.G_p_I.x(), current_state.G_p_I.y(), "192.168.1.100", 8888);
                        Eigen::Quaterniond q(current_state.G_R_I);
                        q.normalize();
                        // 旋转矩阵转旋转向量
                        Eigen::AngleAxisd rotate_vec(q);
                        char str_buffer[100] = {0};
                        // angle, rx,ry,rz
                        sprintf(str_buffer, "%.3f %.3f %.3f %3f", rotate_vec.angle() * kRadianToDegree, rotate_vec.axis().x(), rotate_vec.axis().y(), rotate_vec.axis().z());
                        // LoggerWrite(str_buffer, strlen(str_buffer), "192.168.143.32", 8888);

                        // printf("[imu predict state q] (%f,%f,%f,%f)\n", q.x(), q.y(), q.z(), q.w());
                        // 旋转矩阵转欧拉角
                        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
                        // printf("[imu predict state rpy] (%f,%f,%f)\n", euler[0] * kRadianToDegree, euler[1] * kRadianToDegree, euler[2] * kRadianToDegree);

                        Eigen::Vector3d acc_bias = current_state.acc_bias;
                        Eigen::Vector3d gyro_bias = current_state.gyro_bias;
                        // imu_odometry_file << current_state.timestamp << " ";
                        // imu_odometry_file << current_state.G_p_I.x() << " " << current_state.G_p_I.y() << " " << current_state.G_p_I.z() << " ";
                        // imu_odometry_file << current_state.G_v_I.x() << " " << current_state.G_v_I.y() << " " << current_state.G_v_I.z() << " ";
                        // imu_odometry_file << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
                        // imu_odometry_file << acc_bias.x() << " " << acc_bias.y() << " " << acc_bias.z() << " ";
                        // imu_odometry_file << gyro_bias.x() << " " << gyro_bias.y() << " " << gyro_bias.z() << " ";
                        // imu_odometry_file << std::endl;
                    }
                }
            }
            imu_odometry_file.close();
            gps_pos_file.close();
        }

        void Odometry(ImuReader &imu_reader, GPSReader &gps_reader)
        {
            std::unique_ptr<Localization::Eskf> localizer_ptr_ =
                std::make_unique<Localization::Eskf>(kAccNoise, kGyroNoise,
                                                     kAccBiasRandomWalk, kGyroBiasRandomWalk,
                                                     kArmPos, kGravityNorm, kVelocityMax);
            Localization::State current_state;
            std::fstream eskf_state_file = std::fstream("eskf_state.bag", std::ios::out);
            std::fstream gps_pos_file = std::fstream("gps_pos.bag", std::ios::out);
            // 当前时间
            auto save_time = std::chrono::steady_clock::now();
            auto save_interval = std::chrono::milliseconds(100);

            while (true)
            {
                const Localization::ImuDataPtr imu_data_ptr = ReadImuData(imu_reader);
                if (imu_data_ptr && localizer_ptr_->ProcessImuData(imu_data_ptr))
                {
                    if (std::chrono::steady_clock::now() - save_time > save_interval)
                    {
                        save_time = std::chrono::steady_clock::now();
                        current_state = localizer_ptr_->state();
                        // printf("[eskf state timestamp %f\n", current_state.timestamp);
                        // printf("[eskf state cov] (%f,%f,%f)\n", current_state.cov.diagonal().x(), current_state.cov.diagonal().y(), current_state.cov.diagonal().z());
                        printf("[eskf state pos] (%f,%f,%f)\n", current_state.G_p_I.x(), current_state.G_p_I.y(), current_state.G_p_I.z());
                        // printf("[eskf state vel] (%f,%f,%f)\n", current_state.G_v_I.x(), current_state.G_v_I.y(), current_state.G_v_I.z());
                        // LoggerWriteTrajectory(current_state.G_p_I.x(), current_state.G_p_I.y(), "192.168.1.100",8888);
                        
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
                    }
                }

                const Localization::GpsDataPtr gps_data_ptr = ReadGpsData(gps_reader);
                if (gps_data_ptr && localizer_ptr_->ProcessGpsData(gps_data_ptr))
                {
                    current_state = localizer_ptr_->state();
                    Eigen::Vector3d enu;
                    ConvertLLAToENU(localizer_ptr_->initial_lla(), gps_data_ptr->lla, enu);
                    printf("[gps pos] (%f,%f,%f)\n", enu.x(), enu.y(), enu.z());

                    // LoggerWriteGps(enu.x(), enu.y(), "192.168.1.100", 8888);
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
    eskf_system.Odometry(imu_reader, gps_reader);
    // eskf_system.CollectImuData(imu_reader);
    // eskf_system.ImuOdometry(imu_reader);
    // eskf_system.PrintImuData(imu_reader);

    return 0;
}
