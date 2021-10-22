#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <iostream>

#include <epos2_canopen/master.h>

bool is_shutdown = false;

void canopenThread(epos2_canopen::CanopenMaster *co)
{
    co->startDevice();
    is_shutdown = true;
}

int main(int argc, char* argv[])
{
    std::string can_device = "can0";
    std::string eds_path = "/home/wankun/Projects/archive/2021/acrobot_hybrid/src/robot_drivers/epos2_canopen/config/master.dcf";
    const int master_id = 127;
    const int slave_id = 1;
    epos2_canopen::CanopenMaster co(can_device, eds_path, master_id, slave_id);
    boost::thread co_thr(boost::bind(canopenThread, &co));
    // std::thread co_thr(canopenThread, &co);

    sleep(2);
    double vel = 0.0;
    uint32_t t = 0;
    while (!is_shutdown)
    {
        // co.getVelocity(vel);
        co.getVelocity(vel);
        co.setCurrent(100);
        // std::cout << "Velocity: " << vel << "." << std::endl;
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }
    co_thr.join();

    return 1;
}