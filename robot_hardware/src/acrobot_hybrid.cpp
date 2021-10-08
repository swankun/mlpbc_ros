#include <boost/assign.hpp>
#include <robot_hardware/acrobot_hybrid.h>

namespace robot_hardware
{

AcrobotHybrid::AcrobotHybrid() : epos_device_(nh_)
{
    // Create and register joint interfaces
    ros::V_string joint_names = boost::assign::list_of("theta1")("theta2");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                                &joints_[i].position, 
                                                                &joints_[i].velocity, 
                                                                &joints_[i].effort);
        jstate_interface_.registerHandle(joint_state_handle);
        if (i == 1)
        {
            hardware_interface::JointHandle joint_handle(joint_state_handle, 
                                                        &joints_[i].command);
            jeffort_interface_.registerHandle(joint_handle);
        }
    }
    registerInterface(&jstate_interface_);
    registerInterface(&jeffort_interface_);
    
    // Setup subscriber for serial feedback
    serial_sub_ = nh_.subscribe("teensy_serial", 1, &AcrobotHybrid::serialCallback, this);
    
}


} // namespace
