#include <string>

#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <rosserial_server/serial_session.h>

#include <robot_hardware/acrobot_hybrid.h>

typedef boost::chrono::steady_clock time_source;
typedef std_srvs::TriggerRequest  ResetReq;
typedef std_srvs::TriggerResponse ResetRes;

void controlThread(ros::Rate rate, robot_hardware::AcrobotHybrid* robot, 
                    controller_manager::ControllerManager* cm)
{
  time_source::time_point last_time = time_source::now();

  while (ros::ok())
  {
    // Calculate monotonic time elapsed
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    // time_source::time_point read_begin = time_source::now();
    robot->read();
    // boost::chrono::duration<double> read_elapsed = time_source::now() - read_begin;
    // ROS_INFO_THROTTLE(0.5, "It took %8.6f sec to read from EPOS", read_elapsed.count());
    cm->update(ros::Time::now(), elapsed);
    robot->write();
    rate.sleep();
  }
  robot->disable();
}

bool serviceCb(const ResetReq &req, ResetRes &res, robot_hardware::AcrobotHybrid* robot)
{
  robot->reset();
  res.success = true;
  return true;
}
bool disableCb(const ResetReq &req, ResetRes &res, robot_hardware::AcrobotHybrid* robot)
{
  robot->disableOperation();
  res.success = true;
  return true;
}
bool enableCb(const ResetReq &req, ResetRes &res, robot_hardware::AcrobotHybrid* robot)
{
  robot->enableDevice();
  res.success = true;
  return true;
}

int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "robot_hardware_node");
  ros::NodeHandle n("~");
  robot_hardware::AcrobotHybrid robot;

  // Create the serial rosserial server in a background ASIO event loop.
  std::string port;
  ros::param::param<std::string>("teensy/port", port, "/dev/ttyACM0");
  boost::asio::io_service io_service;
  new rosserial_server::SerialSession(io_service, port, 115200);
  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

  // Background thread for the controls callback.
  controller_manager::ControllerManager cm(&robot);
  int update_rate;
  ros::param::param<int>("control_loop_rate", update_rate, 1000);
  boost::thread(boost::bind(controlThread, ros::Rate(static_cast<double>(update_rate)), &robot, &cm));

  // ROS pub/sub/services for main thread
  ros::ServiceServer service_reset = n.advertiseService<ResetReq,ResetRes>("clear_epos_faults", boost::bind(serviceCb, _1, _2, &robot));
  ros::ServiceServer service_disable = n.advertiseService<ResetReq,ResetRes>("disable_epos", boost::bind(disableCb, _1, _2, &robot));
  ros::ServiceServer service_enable = n.advertiseService<ResetReq,ResetRes>("enable_epos", boost::bind(enableCb, _1, _2, &robot));

  // Foreground ROS spinner for ROS callbacks, including rosserial, diagnostics
  ros::spin();

  return 0;
}