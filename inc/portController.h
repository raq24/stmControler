
#ifndef _PORT_CONTROLLER_
#define _PORT_CONTROLLER_

#include <iostream>

#define PORT_OK 0x00
#define SEND_ERROR 0x01

class PortController
{
public:
    static constexpr const char* STM_PORT = "/dev/ttyACM0";
    PortController(const std::string& port = STM_PORT);
    ~PortController();

    PortController& operator<<(const std::string& data);

private:
    int mPortDesc;
    int errorCode;
};

#endif