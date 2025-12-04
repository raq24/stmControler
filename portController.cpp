
#include "portController.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <exception>

static constexpr const int STM_PORT_ACCESS_TIMEOUT = 8; // seconds

static void configTTY(int fd);

PortController::PortController(const std::string& port)
{  
    if (access(STM_PORT, F_OK) != 0)
    {
        std::cout << "Waiting for stm microcontroller...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        int counter = 0;
        constexpr const int timeout = STM_PORT_ACCESS_TIMEOUT * 10;
        while (access(STM_PORT, F_OK) != 0 && counter < timeout)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            ++counter;
        }
        if (counter >= timeout)
        {
            throw std::runtime_error(
                "Stm port was not registered in the system. stmController stopped.\n"
            );
        }
    }

    std::cout << "Stm port detected\n";

    mPortDesc = open(STM_PORT, O_RDWR | O_NOCTTY);
    if (mPortDesc < 0)
    {
        throw std::runtime_error(
            "Opening stm port: " + std::string(STM_PORT) + " failed with error:\n"
            + std::string(strerror(errno))
        );
    }

    configTTY(mPortDesc);
}

PortController& PortController::operator<<(const std::string& data)
{
    size_t length = data.length();
    auto ptr = data.c_str();
    size_t left = length;
    while (left > 0)
    {
        size_t n = write(mPortDesc, ptr, left);
        if (n < 0) 
        {
            if (errno == EINTR)
                continue;
            errorCode = SEND_ERROR;
            break;
        }
        left -= n;
        ptr += n;
    }
    
    return *this;
}

static void configTTY(int fd)
{
    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        close(fd);
        throw std::runtime_error(
            "Can not setting stm port. Failed with error:\n"
            + std::string(strerror(errno))
        );
    }

    cfmakeraw(&tty); // set tty type: raw

    // set speed
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD);

    // no HW flow control
    tty.c_cflag &= ~CRTSCTS;

    // no SW flow control 
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // no CR/LF conversion
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);
    tty.c_oflag &= ~(OPOST | ONLCR | OCRNL);

    // VMIN/VTIME
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 0.1 s

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        close(fd);
        throw std::runtime_error(
            "Can not setting stm port. Failed with error:\n"
            + std::string(strerror(errno))
        );
    }

    // flush buffers
    tcflush(fd, TCIOFLUSH);
}


PortController::~PortController()
{
    if (mPortDesc)
        close(mPortDesc);
}