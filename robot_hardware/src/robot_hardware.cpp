#include <string>

#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <rosserial_server/serial_session.h>

#include <robot_hardware/acrobot_hybrid.h>

typedef boost::chrono::steady_clock time_source;

void controlThread(ros::Rate rate, robot_hardware::AcrobotHybrid* robot, 
                    controller_manager::ControllerManager* cm)
{
  time_source::time_point last_time = time_source::now();

  while (1)
  {
    // Calculate monotonic time elapsed
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    robot->read();
    cm->update(ros::Time::now(), elapsed);
    robot->write();
    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "robot_hardware_node");
  robot_hardware::AcrobotHybrid robot;

  // Create the serial rosserial server in a background ASIO event loop.
  std::string port;
  ros::param::param<std::string>("~teensy/port", port, "/dev/ttyACM0");
  boost::asio::io_service io_service;
  new rosserial_server::SerialSession(io_service, port, 115200);
  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

  // Background thread for the controls callback.
  ros::NodeHandle controller_nh("/robot_hardware");
  controller_manager::ControllerManager cm(&robot, controller_nh);
  boost::thread(boost::bind(controlThread, ros::Rate(50), &robot, &cm));

  // Foreground ROS spinner for ROS callbacks, including rosserial, diagnostics
  ros::spin();

  return 0;
}