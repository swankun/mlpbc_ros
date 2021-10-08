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
        if (i == 1) // Only the second joint is actuated
        {
            hardware_interface::JointHandle joint_handle(joint_state_handle, 
                                                        &joints_[i].command);
            jeffort_interface_.registerHandle(joint_handle);
        }
    }
    registerInterface(&jstate_interface_);
    registerInterface(&jeffort_interface_);
    
    // Setup subscriber for serial feedback
    ros::NodeHandle teensy_nh(nh_, "teensy");
    std::string serial_topic = teensy_nh.param<std::string>("topic", "teensy_serial");
    serial_sub_ = nh_.subscribe(serial_topic, 1, &AcrobotHybrid::serialCallback, this);
}

AcrobotHybrid::~AcrobotHybrid()
{
    epos_device_.~EposDevice();
}

void AcrobotHybrid::serialCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Update the feedback message pointer to point to the current message. Block
	// until the control thread is not using the lock.
    boost::mutex::scoped_lock lock(serial_feedback_mutex_);
	serial_feedback_ = msg;
}

void AcrobotHybrid::read()
{
    boost::mutex::scoped_lock feedback_msg_lock(serial_feedback_mutex_, boost::try_to_lock);
    if (serial_feedback_ && feedback_msg_lock)
    {
        joints_[0].position = serial_feedback_->position[0];
        joints_[0].velocity = serial_feedback_->velocity[0];
    }
    joints_[1].position = epos_device_.readPosition();
    joints_[1].velocity = epos_device_.readVelocity();
    joints_[1].effort = epos_device_.readCurrent();
}

void AcrobotHybrid::write()
{
    epos_device_.writeCurrent(joints_[1].command);
}


} // namespace
