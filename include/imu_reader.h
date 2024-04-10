#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <memory.h>
#include <array>
#include <atomic>
#include <thread>
#include <sys/time.h>
#include <math.h>
#include "serial.h"

// 字节对齐
#pragma pack(1)
struct ImuDataHead
{
    uint16_t head_code;
    uint8_t length;
    uint32_t type;
    uint8_t data[];
};

struct ImuOriginData
{
    union
    {
        float value;
        uint8_t bytes[4];
    } timestamp;
    union
    {
        float value;
        uint8_t bytes[4];
    } gyro[3];
    union
    {
        float value;
        uint8_t bytes[4];
    } accel[3];
    union
    {
        float value;
        uint8_t bytes[4];
    } mag[3];
};
#pragma pack()

class ImuReader
{
public:
    constexpr static int HEAD_CODE_SIZE = sizeof(uint16_t);
    constexpr static int TYPE_SIZE = sizeof(uint32_t);
    constexpr static int CHECK_CODE_SIZE = sizeof(uint16_t);
    constexpr static int ORIGIN_DATA_SIZE = sizeof(ImuOriginData);

private:
    SerialPort serial;
    std::atomic<bool> _running{false};
    std::atomic<bool> _empty{true};
    std::atomic<ImuOriginData> _data;
    std::atomic<float> _time_diff{-1.0};

public:
    ImuReader(std::string port, int baudrate) : serial(port, baudrate) {}
    int find_head(uint8_t *buf, int start, int end)
    {
        const uint8_t head_code[2] = {0xaa, 0x55};
        for (int i = start; i < end - 1; i++)
        {
            if (buf[i] == head_code[0] && buf[i + 1] == head_code[1])
            {
                return i;
            }
        }
        return end;
    }

    bool getData(ImuOriginData &data)
    {
        if (_empty)
        {
            // printf("imu data empty\n");
            return false;
        }
        _empty = true;
        data = _data.load();
        return true;
    }

    void DisableReadThread()
    {
        _running = false;
    }
    void EnableReadThread();

    ~ImuReader()
    {
        DisableReadThread();
    }

private:
    void setData(const ImuOriginData &data)
    {
        _data.store(data);
        _empty = false;
    }
    void print_bytes(uint8_t *data, int size)
    {
        for (int i = 0; i < size; i++)
        {
            printf("0x%02x ", data[i]);
        }
        printf("\n");
    }

    bool check_crc(uint8_t *data, int size, uint16_t crc_code);
    
    float UpdateImuTimeDiff(float imu_timestamp)
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        float timestamp = tv.tv_sec % (24 * 3600) + tv.tv_usec * 0.000001f;
        float diff = timestamp - imu_timestamp;

        if (_time_diff > 0.0)
        {
            timestamp = imu_timestamp + _time_diff.load();
        }
        _time_diff.store(diff);
        return timestamp;
    }
};
