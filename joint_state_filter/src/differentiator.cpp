#include <joint_state_filter/differentiator.h>

namespace joint_state_filter
{

Differentiator::Differentiator(ros::NodeHandle &nh)
{
    int order, sampling_frequency, cutoff_frequency;
    
    nh.param<int>("order", order, 8);
    ROS_INFO("[joint_state_filter]: order = %d", order);

	nh.param<int>("sampling_frequency", sampling_frequency, 1000);
    ROS_INFO("[joint_state_filter]: sampling_frequency = %d", sampling_frequency);

    nh.param<int>("cutoff_frequency", cutoff_frequency, 10);
    ROS_INFO("[joint_state_filter]: cutoff_frequency = %d", cutoff_frequency);
    period_ = 1.0 / (double)sampling_frequency;

    if ( !nh.param<ros::V_string>("joint_names", joint_names_, {"joint"}) )
    {
        ROS_FATAL("joint names must be provided in rosparam");
    }
    for (auto name = joint_names_.begin(); name != joint_names_.end(); ++name)
    {
        ROS_INFO("[joint_state_filter]: joint_names = %s", name->c_str());
    }
    for (int i = 0; i < 3; i++)
    {
        Iir::Butterworth::LowPass<MAX_FILTER_ORDER> f;
        f.setup(order, sampling_frequency, cutoff_frequency);
        filters_.push_back(f);
    }
    
    jstate_sub_ = nh.subscribe("/joint_states", 1, &Differentiator::jointStateCb, this);
    filter_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states_filtered", 1);
}


Differentiator::~Differentiator()
{}


void Differentiator::jointStateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
    sensor_msgs::JointState jstate_msg;
    jstate_msg.header = msg->header;

    int i = 0;
    for (auto name = msg->name.begin(); name != msg->name.end(); ++name, ++i)
    {
        if (name->compare(joint_names_.front()) != 0)
        {
            continue;
        }
        jstate_msg.name.push_back(name->c_str());
        
        const double pos = msg->position.front();
        const double filt_pos = filters_[0].filter(pos);
        jstate_msg.position.push_back(filt_pos);
    
        const double delta_pos = (1/period_) * ( pos - old_states_.position );
        const double filt_vel = filters_[1].filter(delta_pos);
        jstate_msg.velocity.push_back(filt_vel);

        const double delta_vel = (1/period_) * ( filt_vel - old_states_.velocity );
        const double filt_acc = filters_[2].filter(delta_vel);
        jstate_msg.effort.push_back(filt_acc);
        filter_pub_.publish(jstate_msg);

        old_states_.position = pos;
        old_states_.velocity = filt_vel;
    }
}

}