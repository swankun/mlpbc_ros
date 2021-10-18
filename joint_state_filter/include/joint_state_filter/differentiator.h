#ifndef JOINT_STATE_FILTER_DIFFERENTIATOR_H
#define JOINT_STATE_FILTER_DIFFERENTIATOR_H

#define MAX_FILTER_ORDER 8

#include <Iir.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace joint_state_filter
{

class Differentiator
{
    public:
        Differentiator(ros::NodeHandle &nh);
        ~Differentiator();

        void jointStateCb(const sensor_msgs::JointState::ConstPtr &msg);

    private:
        double period_;
        std::vector<Iir::Butterworth::LowPass<MAX_FILTER_ORDER>> filters_;
        ros::V_string joint_names_;
        ros::Subscriber jstate_sub_;
        ros::Publisher  filter_pub_;
        struct States {
            double position, velocity, acceleration;
            States() : position(0.0), velocity(0.0), acceleration(0.0) {}
        } old_states_;


}; // class Differentiator

} // namespace joint_state_filter

#endif // JOINT_STATE_FILTER_DIFFERENTIATOR_H