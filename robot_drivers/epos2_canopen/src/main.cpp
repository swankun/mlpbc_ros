#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <iostream>

#include <epos2_canopen/master.h>


int main(int argc, char* argv[])
{
    std::string can_device = "vcan0";
    std::string eds_path = "/home/wankun/Projects/archive/2021/acrobot_hybrid/src/robot_drivers/epos2_canopen/config/tutorial-master.dcf";
    const int master_id = 1;
    const int slave_id = 2;
    epos2_canopen::CanopenMaster co(can_device, eds_path, master_id, slave_id);
    boost::thread co_thr( [&]() { co.startDevice(); } );

    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    double vel = 0.0;
    uint32_t t = 0;
    // while (!is_shutdown)
    for (int i=1; i<10; i++)
    {
        // co.getVelocity(vel);
        co.getVelocity(vel);
        // co.setCurrent(100);
        // std::cout << "Velocity: " << vel << "." << std::endl;
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }
    co.stopDevice();
    co_thr.join();
    std::cout << "Press Enter to exit." << std::endl;
    // std::cin.get();
    return 0;
}