#include "gps_reader.h"
#include <vector>
#include <thread>
#include "nmea.h"

//$GPGGA,014434.70,3817.13334637,N,12139.72994196,E,4,07,1.5,6.571,M,8.942,M,0.7,0016*79
bool GPSReader::ParseGGA(const std::string buf, GpsOriginData &data)
{
    std::vector<std::string> str_vecs;
    str_vecs.reserve(kGGACheckSum);
    int start = 0, pos = 0;
    while ((pos = buf.find(',', start)) != std::string::npos) // 根据,分割字符串
    {
        str_vecs.push_back(buf.substr(start, pos - start));
        start = pos + 1;
    }

    str_vecs.push_back(buf.substr(start));
    if (str_vecs.size() != kGGACheckSum)
    {
        return false;
    }
    // 时间 hhmmss.sss
    if (str_vecs[kGGATime].empty())
    {
        return false;
    }
    double timestamp = std::stod(str_vecs[kGGATime]);
    int hour = timestamp / 10000;
    int minute = (timestamp - hour * 10000) / 100;
    double second = timestamp - hour * 10000 - minute * 100;
    timestamp = hour * 3600 + minute * 60 + second;

    // 纬度 ddmm.mmmm
    if (str_vecs[kGGALatitude].empty() || str_vecs[kGGANorS].empty())
    {
        return false;
    }
    double latitude = std::stod(str_vecs[kGGALatitude]);
    int degree = latitude / 100;
    latitude = degree + (latitude - degree * 100) / 60;
    // N/S
    if (str_vecs[kGGANorS] == "S")
    {
        latitude = -latitude;
    }

    // 经度 dddmm.mmmm
    if (str_vecs[kGGALongitude].empty() || str_vecs[kGGAEorW].empty())
    {
        return false;
    }
    double longitude = std::stod(str_vecs[kGGALongitude]);
    degree = longitude / 100;
    longitude = degree + (longitude - degree * 100) / 60;
    // E/W
    if (str_vecs[kGGAEorW] == "W")
    {
        longitude = -longitude;
    }
    // 海拔和椭球高度
    if (str_vecs[kGGAAltitude].empty() || str_vecs[kGGAEllipsoideHeight].empty())
    {
        return false;
    }
    // 海拔高度
    double altitude = std::stod(str_vecs[kGGAAltitude]);
    // // 椭球相对于海平面的高度
    // double ellipsoide_height = std::stod(str_vecs[kGGAEllipsoideHeight]);
    // // 相对于椭球面的高度
    // altitude = altitude - ellipsoide_height;

    // 0初始化， 1单点定位， 2码差分， 3无效PPS， 4固定解， 5浮点解， 6正在估算 7，人工输入固定值， 8模拟模式， 9WAAS差分
    if (str_vecs[kGGAQuality].empty())
    {
        return false;
    }
    int quality = std::stoi(str_vecs[kGGAQuality]);
    SetGpsOriginData(data, timestamp, latitude, longitude, altitude, quality, "$GGA");

    return true;
}

//$GPRMC,121252.000,A,3958.3032,N,11629.6046,E,15.15,359.95,070306,,,A*54
bool GPSReader::ParseRMC(const std::string buf, GpsOriginData &data)
{
    std::vector<std::string> str_vecs;
    str_vecs.reserve(kRMCCheckSum);
    int start = 0, pos = 0;
    while ((pos = buf.find(',', start)) != std::string::npos)
    {
        str_vecs.push_back(buf.substr(start, pos - start));
        start = pos + 1;
    }
    str_vecs.push_back(buf.substr(start));
    if (str_vecs.size() != kRMCCheckSum)
    {
        return false;
    }
    // 时间 hhmmss.sss
    if (str_vecs[kRMCTime].empty())
    {
        return false;
    }
    double timestamp = std::stod(str_vecs[kRMCTime]);
    int hour = timestamp / 10000;
    int minute = (timestamp - hour * 10000) / 100;
    double second = timestamp - hour * 10000 - minute * 100;
    timestamp = hour * 3600 + minute * 60 + second;

    // 状态 A有效，V无效
    if (str_vecs[kRMCStatus].empty())
    {
        return false;
    }
    int status = str_vecs[kRMCStatus] == "A" ? 1 : 0;

    // 纬度 ddmm.mmmm
    if (str_vecs[kRMCLatitude].empty() || str_vecs[kRMCNorS].empty())
    {
        return false;
    }
    double latitude = std::stod(str_vecs[kRMCLatitude]);
    int degree = latitude / 100;
    latitude = degree + (latitude - degree * 100) / 60;
    // N/S
    if (str_vecs[kRMCNorS] == "S")
    {
        latitude = -latitude;
    }

    // 经度 dddmm.mmmm
    if (str_vecs[kRMCLongitude].empty() || str_vecs[kRMCEorW].empty())
    {
        return false;
    }
    double longitude = std::stod(str_vecs[kRMCLongitude]);
    degree = longitude / 100;
    longitude = degree + (longitude - degree * 100) / 60;
    // E/W
    if (str_vecs[kRMCEorW] == "W")
    {
        longitude = -longitude;
    }
    SetGpsOriginData(data, timestamp, latitude, longitude, 0, status, "$RMC");
    return true;
}

void GPSReader::EnableReadThread()
{
    if (_running)
    {
        printf("gps reader already running\n");
        return;
    }
    if (!_serial.open())
    {
        printf("open serial port failed\n");
        return;
    }
    _running = true;
    std::thread _run(
        [this]
        {
            int start = 0, end = 0;
            const int MAX_LEN = 1024;
            const int MESSAGE_LEN = 256;
            char buf[MAX_LEN] = {0};
            std::string head_code;
            int max_code_size = 10;
            char message[MESSAGE_LEN] = {0};
            while (_running)
            {
                int n = _serial.read(buf + end, MAX_LEN / 2);
                if (n <= 0)
                {
                    usleep(1000);
                    continue;
                }
                // printf("read %d gps bytes\n", n);
                end += n;
                start = find_dollar(buf, start, end, head_code, max_code_size);
                int pack_end;

                while ((pack_end = find_star(buf, start, end)) < end)
                {
                    if (!check(buf, &start, pack_end, head_code, max_code_size))
                    {
                        printf("check failed\n");
                        start = find_dollar(buf, pack_end + 1, end, head_code, max_code_size);
                        continue;
                    }

                    memset(message, 0, MESSAGE_LEN);
                    memcpy(message, buf + start, pack_end - start);
                    // parse GGA
                    // if (head_code.compare("$GPGGA") == 0 || head_code.compare("$GNGGA") == 0)
                    // {
                    //     printf("message: %s\n", message);
                    //     GpsOriginData data;
                    //     if (ParseGGA(message, data))
                    //     {
                    //         data.timestamp = UpdateGpsTimeDiff(data.timestamp);
                    //         setData(data);
                    //         // printf("latitude: %f, longitude: %f, altitude: %f, timestamp: %f, quality: %d\n", data.lat, data.lon, data.alt, data.timestamp, data.status);
                    //     }
                    //     else
                    //     {
                    //         printf("parse GGA failed\n");
                    //     }
                    // }

                    if (head_code.compare("$GPRMC") == 0 || head_code.compare("$GNRMC") == 0)
                    {
                        GpsOriginData data;
                        if (ParseRMC(message, data))
                        {
                            data.timestamp = UpdateGpsTimeDiff(data.timestamp);
                            setData(data);
                            // printf("gps timestamp: %.6f\n", data.timestamp);
                        }
                        else
                        {
                            printf("parse RMC failed\n");
                        }
                    }

                    start = find_dollar(buf, pack_end + 1, end, head_code, max_code_size);
                }

                if (start >= end)
                {
                    start = 0, end = 0;
                }
                else if (start >= MAX_LEN / 2)
                {
                    memcpy(buf, buf + start, end - start);
                    end -= start;
                    start = 0;
                }
            }
            // close the serial port
            _serial.close();
        });
    _run.detach();
}

int GPSReader::find_dollar(const char *buf, int start, int end, std::string &head_code, int code_max_size)
{
    int dollar_index = -1;
    for (int i = start; i < end; ++i)
    {
        if (buf[i] == '$') // buf char type
        {
            dollar_index = i;
            // find the head code
            for (int j = i + 1; j < end; ++j)
            {
                if (buf[j] == ',')
                {
                    head_code = std::string(buf + i, buf + j);
                    return i;
                }
                else if (j - i > code_max_size || buf[j] == '$')
                {
                    dollar_index = -1;
                    break;
                }
            }
        }
    }
    if (dollar_index == -1)
    {
        return end;
    }
    return dollar_index;
}

int GPSReader::find_star(const char *buf, int start, int end)
{
    const int check_code_size = 2;
    for (int i = start; i < end - check_code_size; ++i)
    {
        if (buf[i] == '*')
        {
            return i;
        }
    }
    return end;
}
