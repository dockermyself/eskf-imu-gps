#include "imu_reader.h"

void ImuReader::EnableReadThread()
{
    if (_running)
    {
        printf("imu reader already running\n");
    }
    if (!serial.open())
    {
        printf("open serial failed\n");
        return;
    }
    std::thread _run(
        [this]
        {
            int buffer_start = 0, buffer_end = 0;
            const int buffer_size = 1024;
            uint8_t buf[buffer_size] = {0};
            _running = true;
            while (_running)
            {
                int n = serial.read((char *)buf + buffer_end, buffer_size / 2);
                if (n <= 0)
                {
                    usleep(1000);
                    continue;
                }
                // printf("read %d imu bytes\n", n);
                buffer_end += n;
                buffer_start = find_head(buf, buffer_start, buffer_end);

                while (buffer_start + sizeof(ImuDataHead) < buffer_end)
                {
                    ImuDataHead *head = (ImuDataHead *)(buf + buffer_start);
                    int buffer_size = buffer_end - buffer_start;
                    // check code 2 bytes
                    if (head->length + sizeof(ImuDataHead) + CHECK_CODE_SIZE - TYPE_SIZE > buffer_size)
                    {
                        break;
                    }
                    // check length
                    if (head->length != ORIGIN_DATA_SIZE + TYPE_SIZE)
                    {
                        buffer_start = find_head(buf, buffer_start + HEAD_CODE_SIZE, buffer_end);
                        continue;
                    }
                    int package_size = head->length + sizeof(ImuDataHead) + HEAD_CODE_SIZE - TYPE_SIZE;
                    if (package_size > buffer_size)
                    {
                        break;
                    }

                    uint16_t *code = (uint16_t *)(head->data + ORIGIN_DATA_SIZE);

                    if (!check_crc(buf + HEAD_CODE_SIZE, buffer_end - buffer_start - HEAD_CODE_SIZE - CHECK_CODE_SIZE, *code))
                    {
                        printf("check sum failed\n");
                        buffer_start = find_head(buf, buffer_start + HEAD_CODE_SIZE, buffer_end);
                        continue;
                    }
                    // printf("buffer_start: %d, buffer_end: %d\n", buffer_start, buffer_end);

                    // parse data
                    ImuOriginData *data = (ImuOriginData *)(head->data);
                    uint32_t *timestamp = (uint32_t *)(data->timestamp.bytes);
                    data->timestamp.value = *timestamp * 0.000001f;

                    data->timestamp.value = UpdateImuTimeDiff(data->timestamp.value);
                    
                    setData(*data);
                    // printf("imu timestamp: %.6f\n",data->timestamp.value);
              
                    buffer_start += head->length + sizeof(ImuDataHead) + CHECK_CODE_SIZE;
                    buffer_start = find_head(buf, buffer_start, buffer_end);
                }

                if (buffer_start >= buffer_end)
                {
                    buffer_start = buffer_end = 0;
                }
                else if (buffer_start > buffer_size / 2)
                {
                    memcpy(buf, buf + buffer_start, buffer_end - buffer_start);
                    buffer_end -= buffer_start;
                    buffer_start = 0;
                }
            }
            serial.close();
        });
    _run.detach();
}

bool ImuReader::check_crc(uint8_t *data, int size, uint16_t crc_code)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < size; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if ((crc & 1) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc == crc_code;
}