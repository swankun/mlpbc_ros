#ifndef EPOS2_CANOPEN_MASTER_ROS_H
#define EPOS2_CANOPEN_MASTER_ROS_H

#define RPM_TO_RADS 0.10471975511965977

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>

#include <epos2_canopen/master.h>

namespace epos2_canopen
{

class Epos2Ros
{

    public:
        Epos2Ros(ros::NodeHandle &nh);
        ~Epos2Ros();
        const double readPosition();
        const double readVelocity();
        const double readCurrent();
        void writeCurrent(const double cmd);
        void disableDevice();

    private:
        std::unique_ptr<CanopenMaster> ptr_co_;
        std::unique_ptr<boost::thread> ptr_co_thr_;
        double rad_=0, rpm_=0, milliamps_=0;
        bool is_running = false;
        void canopen_task();
        double pulses_to_rad_;

}; // class Epos2Ros

} // namespace

#endif // EPOS2_CANOPEN_MASTER_ROS_H