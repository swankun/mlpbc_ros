#ifndef ROBOT_HARDWARE_EPOS_DEVICE_H
#define ROBOT_HARDWARE_EPOS_DEVICE_H

#define RPM_TO_RADS 0.10471975511965977

#include <ros/ros.h>
#include <maxon_epos_driver/Device.hpp>

namespace robot_hardware
{

/**
 * @brief Class for handling reading from and sending to Maxon EPOS device.
 * Only support incremental encoder and current control mode.
 * 
 */
class EposDevice
{
    public:

        EposDevice(ros::NodeHandle &nh);
        ~EposDevice();
        
        /**
         * @brief Clear EPOS errors
         * 
         */
        void clearDeviceErrors();

        double readPosition();
        double readVelocity();
        double readCurrent();
        void writeCurrent(const double cmd);
        void disableDevice();

    private:

        /**
         * @brief Epos NodeHandle
         * 
         */
        NodeHandle epos_handle_;

        double pulses_to_rad_;
        double last_command_;

}; // class EposDevice

} // namespace robot_hardware

#endif // ROBOT_HARDWARE_EPOS_DEVICE_H
