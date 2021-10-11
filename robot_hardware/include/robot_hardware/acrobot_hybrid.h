#ifndef ROBOT_HARDWARE_ACROBOT_HYBRID_H
#define ROBOT_HARDWARE_ACROBOT_HYBRID_H

#include <boost/thread.hpp>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <robot_hardware/epos_device.h>

namespace robot_hardware
{

class AcrobotHybrid : public hardware_interface::RobotHW
{

    public:
        /**
         * @brief Construct a new Acrobot Hybrid object
         * 
         */
        AcrobotHybrid();
        
        /**
         * @brief Destroy the Acrobot Hybrid object
         * 
         */
        ~AcrobotHybrid();

        
        /**
         * @brief Read feedback from hardware
         * 
         */
        void read();

        /**
         * @brief Write commands to hardware
         * 
         */
        void write();

        /**
         * @brief Disable the hardware 
         * 
         */
        void disable();


    protected:

        /**
         * @brief Callback for theta1 encoder feedback, which is received via 
         * rosserial messages from Teensy
         * 
         * @param msg The message
         */
        void serialCallback(const sensor_msgs::JointState::ConstPtr& msg);

        /**
         * @brief Handle to ROS Node that spins rosserial 
         * 
         */
        ros::NodeHandle nh_;
        
        /**
         * @brief Subscriber object for rosserial message
         * 
         */
        ros::Subscriber serial_sub_;
        
        /**
         * @brief Interface for obtaining feedback of robot's joints 
         * 
         */
        hardware_interface::JointStateInterface jstate_interface_;

        /**
         * @brief Interface for sending effort comands to robot's joints
         * 
         */
        hardware_interface::EffortJointInterface jeffort_interface_;

        /**
         * @brief Struct for holding each joint's information
         * 
         */
        struct Joint
        {
            double position, velocity, effort, command;
            Joint() : position(0), velocity(0), effort(0), command(0) {}
        } joints_[2];

        /**
         * @brief JointState message for holding serial feedback from Teensy
         * 
         */
        sensor_msgs::JointState::ConstPtr serial_feedback_;

        /**
         * @brief Mutex for guarding the feedback message
         * 
         */
        boost::mutex serial_feedback_mutex_;

        /**
         * @brief  EposDevice object representing the EPOS2 controller
         * 
         */
        EposDevice epos_device_;


}; // class AcrobotHybrid

}  // namespace robot_hardware

#endif // ROBOT_HARDWARE_ACROBOT_HYBRID_H
