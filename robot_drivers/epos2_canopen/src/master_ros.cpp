#include<epos2_canopen/master_ros.h>

namespace epos2_canopen
{

Epos2Ros::Epos2Ros(ros::NodeHandle &nh)
{
    ros::NodeHandle epos_nh(nh,"epos_canopen");
    std::string eds_file;
    if (!epos_nh.getParam("eds_file", eds_file))
    {
        ROS_ERROR_STREAM("Must provide EDS file path as rosparam");
    }
    std::string eds_bin = epos_nh.param<std::string>("eds_bin", "");
    std::string can_device = epos_nh.param<std::string>("can_device", "can0");
    const unsigned short master_id(epos_nh.param("master_id", 255));
    const unsigned short slave_id(epos_nh.param("slave_id", 1));
    ptr_co_ = std::make_unique<CanopenMaster>(can_device, eds_file, eds_bin, master_id, slave_id);
    ptr_co_thr_ = std::make_unique<boost::thread>( [&](){ptr_co_->startDevice();} );
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    const int resolution(epos_nh.param("pulses_per_rev", 4096));
    pulses_to_rad_ = 4.0 * resolution / 2.0 / M_PI;
}

Epos2Ros::~Epos2Ros()
{
    disableDevice();
}

const double Epos2Ros::readPosition()
{
    ptr_co_->getPosition(rad_);
    return rad_ / pulses_to_rad_;
}

const double Epos2Ros::readVelocity()
{
    ptr_co_->getVelocity(rpm_);
    return rpm_ * RPM_TO_RADS;
}

const double Epos2Ros::readCurrent()
{
    ptr_co_->getCurrent(milliamps_);
    return milliamps_ / 1000.0;
}

void Epos2Ros::writeCurrent(const double cmd)
{
    ptr_co_->setCurrent(cmd*1000.0);
}

void Epos2Ros::clearErrors()
{
    ptr_co_->setCurrent(0.0);
    ptr_co_->clearFaults();
}

void Epos2Ros::enableDevice()
{
    ptr_co_->enableDevice();
}

void Epos2Ros::disableOperation()
{
    ptr_co_->disableOperation();
}

void Epos2Ros::disableDevice()
{
    ptr_co_->setCurrent(0.0);
    ptr_co_->stopDevice();
    ptr_co_thr_->join();
}

}
