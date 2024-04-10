#ifndef _GPS_READER_H_
#define _GPS_READER_H_
#include <iostream>
#include <atomic>
#include <string>
#include <memory.h>
#include <sys/time.h>
#include "serial.h"

// GGA和RMC的数据结构
struct GpsOriginData
{
    char code[10];
    double timestamp;
    double lat;
    double lon;
    double alt;
    int status;
};

class GPSReader
{
    SerialPort _serial;
    std::atomic<bool> _running = {false};
    std::atomic<bool> _empty = {true};
    std::atomic<GpsOriginData> _data = {GpsOriginData()};
    std::atomic<double> _time_diff = {-1.0};

public:
    GPSReader(std::string port, int baudrate) : _serial(port, baudrate) {}
    void EnableReadThread();
    void DisableReadThread()
    {
        if (_running)
        {
            _running = false;
        }
    }
    bool getData(GpsOriginData &data)
    {
        if (_empty)
        {
            return false;
        }
        _empty = true;
        data = this->_data.load();
        return true;
    }

    ~GPSReader()
    {
        if (_running)
        {
            _running = false;
        }
    }

private:
    void setData(const GpsOriginData &data)
    {
        _data.store(data);
        _empty = false;
    }

    int find_dollar(const char *buf, int start, int end, std::string &head_code, int code_max_size = 10);

    int find_star(const char *buf, int start, int end);

    //$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    int check(const char *buf, int len) // buf 代表$后的数据
    {
        int sum = 0;
        for (int i = 0; i < len; ++i)
        {
            sum ^= buf[i];
        }
        return sum;
    }

    bool check(const char *buf, int *pack_start, int pack_end, std::string &head_code, int max_code_size = 10)
    {
        // pack_end 代表*的位置,pack_start代表$的位置
        while (*pack_start < pack_end)
        {
            char str[3] = {*(buf + pack_end + 1), *(buf + pack_end + 2), '\0'};
            if (check(buf + *pack_start + 1, pack_end - *pack_start - 1) == strtol(str, NULL, 16))
            {
                return true;
            }
            *pack_start = find_dollar(buf, *pack_start + 1, pack_end, head_code, max_code_size);
        }
        return false;
    }
    void print_bytes(const char *buf, int len)
    {
        for (int i = 0; i < len; ++i)
        {
            printf("%02x ", buf[i]);
        }
        printf("\n");
    }

    bool ParseGGA(const std::string buf, GpsOriginData &lla);
    bool ParseRMC(const std::string buf, GpsOriginData &lla);
    void SetGpsOriginData(GpsOriginData &lla, double _timestamp, double _lat, double _lon, double _alt, int _status, const std::string &_code)
    {
        lla.timestamp = _timestamp;
        lla.lat = _lat;
        lla.lon = _lon;
        lla.alt = _alt;
        lla.status = _status;
        if (_code.size() < sizeof(lla.code))
        {
            strcpy(lla.code, _code.c_str());
        }
    }

    float UpdateGpsTimeDiff(float gps_timestamp)
    {
        // 获取当前时间
        struct timeval tv;
        gettimeofday(&tv, NULL);
        float timestamp = tv.tv_sec % (24 * 3600) + tv.tv_usec * 0.000001f;
        float diff = timestamp - gps_timestamp;
        // printf("gps time delay: %f\n", diff);
        if (_time_diff > 0.0)
        {
            timestamp = gps_timestamp + _time_diff.load();
        }
        _time_diff.store(diff);
        return timestamp;
    }
};

#endif
