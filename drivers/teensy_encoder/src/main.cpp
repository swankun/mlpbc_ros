#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>

#include <Encoder.h>
#include <filters.h>

#define PUBLISH_PERIOD_MICRO 1000

typedef std_srvs::TriggerRequest  ResetReq;
typedef std_srvs::TriggerResponse ResetRes;

// Encoder
Encoder encoder(3, 2);
const float encoder_resolution = 1024;
float old_position = -999;

// Lowpass filter
const float cutoff_freq   = 15.0;     // Cutoff frequency in Hz
const float sampling_time = (float)PUBLISH_PERIOD_MICRO/1e6;    // Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)
Filter lpf(15.0, sampling_time, order);

// ROS message
sensor_msgs::JointState jstate;
char *_jstate_name[] = {"joint1"};
float _jstate_pos[1] = {0};
float _jstate_vel[1] = {0};
float _jstate_eff[1] = {0};

// ROS node
ros::NodeHandle nh;

// Publisher
ros::Publisher  joint_state_publisher("/serial_encoder/joint_state", &jstate);
void publishState()
{
  float current_position =  (float)encoder.read() / 4.0 / encoder_resolution * 2.0 * M_PI;
  float delta = ( current_position - old_position ) / sampling_time;
  old_position = current_position;

  _jstate_pos[0] = current_position;
  _jstate_vel[0] = lpf.filterIn(delta);
  
  jstate.position[0] = _jstate_pos[0];
  jstate.velocity[0] = _jstate_vel[0];
  jstate.effort[0] = 0.0;
  jstate.header.stamp = nh.now();
  joint_state_publisher.publish( &jstate );
}

// Subscribers and Callbacks
void resetSrvCb(const ResetReq &req, ResetRes &res)
{
  encoder.write(0);
  lpf.flush();
  res.success = true;
  res.message = "Encoder state reset successfully.";
}
ros::ServiceServer<ResetReq, ResetRes> reset_service_server("/serial_encoder/reset", &resetSrvCb);


// Timer
elapsedMicros last_spin;

void setup()
{ 
  nh.initNode();
  jstate.name            = _jstate_name;
  jstate.position        = _jstate_pos;
  jstate.velocity        = _jstate_vel;
  jstate.effort          = _jstate_eff;
  jstate.name_length     = 1;
  jstate.position_length = 1;
  jstate.velocity_length = 1;
  jstate.effort_length   = 1;
  nh.advertise(joint_state_publisher);
  nh.advertiseService(reset_service_server);
}


void loop()
{ 
  // Publish state
  if (last_spin >= PUBLISH_PERIOD_MICRO) {
    publishState();
    nh.spinOnce();
    last_spin -= PUBLISH_PERIOD_MICRO;
  }
  
}
