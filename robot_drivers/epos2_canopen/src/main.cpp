#include <boost/thread.hpp>
#include <iostream>

#include <epos2_canopen/master.h>

void canopenThread(epos2_canopen::CanopenMaster *co)
{
    co->startDevice();
}

int main(int argc, char* argv[])
{
    std::string can_device = "vcan0";
    std::string eds_path = "/home/wankun/Projects/IWP/ros/src/robot_drivers/epos2_canopen/config/master.dcf";
    epos2_canopen::CanopenMaster co(can_device, eds_path, 1, 2);
    boost::thread(boost::bind(canopenThread, &co));
    // std::thread co_thr(canopenThread, &co);

    sleep(5);
    double vel = 0.0;
    uint32_t t = 0;
    while (true)
    {
        // co.getVelocity(vel);
        // co.getVelocity(vel);
        // std::cout << "Velocity: " << vel << "." << std::endl;
        co.getDummy();
        sleep(1);
    }

    return 0;
}