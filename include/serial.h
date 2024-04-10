#ifndef _SERIAL_H_
#define _SERIAL_H_
#include <iostream>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class SerialPort
{
    int fd;
    std::string port;
    int baudrate;

public:
    SerialPort(std::string port, int baudrate) : port(port), baudrate(baudrate) {}
    bool open()
    {
        // fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        fd = ::open(port.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1)
        {
            std::cout << "open port failed" << std::endl;
            return false;
        }
        struct termios options;
        tcgetattr(fd, &options);
        cfsetispeed(&options, baudrate);
        cfsetospeed(&options, baudrate);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_oflag &= ~OPOST;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(INPCK | ICRNL | IGNCR);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_cc[VTIME] = 0;
        options.c_cc[VMIN] = 1;
        if(tcsetattr(fd, TCSANOW, &options) != 0){
            return false;
        }
        return true;
    }

    void close()
    {
        ::close(fd);
    }

    int read(char *buf, int len)
    {
        return ::read(fd, buf, len);
    }

    int write(char *buf, int len)
    {
        return ::write(fd, buf, len);
    }
};

#endif