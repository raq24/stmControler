#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <cerrno>

#include "portController.h"

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    std::cout << "Start stmControler process...\n";

    PortController port;

    port << "fan on\n";

    std::this_thread::sleep_for(7000ms);

    port << "fan off\n";

    return 0;
}