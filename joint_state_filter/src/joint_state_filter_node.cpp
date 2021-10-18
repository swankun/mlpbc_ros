// Usage Examples
//
// This shows you how to operate the filters
//

// This is the only include you need
#include <Iir.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <joint_state_filter/differentiator.h>

int main(int argc, char **argv){
	
	ros::init(argc, argv, "joint_state_filter_node");
	ros::NodeHandle nh("~");

	int sampling_frequency, cutoff_frequency;
	ros::param::param<int>("sampling_frequency", sampling_frequency, 1000);
	ros::param::param<int>("cutoff_frequency", cutoff_frequency, 10);

	joint_state_filter::Differentiator diff(nh);

	ros::spin();
}
