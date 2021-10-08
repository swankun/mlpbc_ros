#include <robot_hardware/epos_device.h>

namespace robot_hardware
{

EposDevice::EposDevice(ros::NodeHandle &nh) : 
    pulses_to_rad_(1.0), last_command_(0.0)
{
    // Create device handle
    ros::NodeHandle epos_nh(nh,"epos");
    const DeviceInfo device_info(epos_nh.param<std::string>("device", "EPOS2"),
                                epos_nh.param<std::string>("protocol_stack", "MAXON SERIAL V2"),
                                epos_nh.param<std::string>("interface", "USB"),
                                epos_nh.param<std::string>("port", "USB0"));
    const unsigned short node_id(epos_nh.param("node_id", 1));
    epos_handle_ = HandleManager::CreateEposHandle(device_info, node_id);
    VCS_NODE_COMMAND_NO_ARGS(SetDisableState, epos_handle_);
    clearDeviceErrors();

    // Setup baud rate and comm timeout
    const unsigned int baudrate(epos_nh.param("baudrate", 0));
    const unsigned int timeout(epos_nh.param("timeout", 0));
    if (baudrate > 0 && timeout > 0) {
        VCS_COMMAND(SetProtocolStackSettings, epos_handle_.ptr.get(), baudrate, timeout);
    } else {
        unsigned int current_baudrate, current_timeout;
        VCS_COMMAND(GetProtocolStackSettings, epos_handle_.ptr.get(), &current_baudrate, &current_timeout);
        VCS_COMMAND(SetProtocolStackSettings, epos_handle_.ptr.get(),
                baudrate > 0 ? baudrate : current_baudrate,
                timeout > 0 ? timeout : current_timeout);
    }

    // Set control mode to current control
    VCS_NODE_COMMAND_NO_ARGS(ActivateCurrentMode, epos_handle_);

    // Encoder configuration
    ros::NodeHandle encoder_nh(epos_nh, "encoder");
    const int type(encoder_nh.param("type", 0)); // 1: INC 3CH, 2: INC 2CH,
    if (type == 1 || type == 2) 
    {
        VCS_NODE_COMMAND(SetSensorType, epos_handle_, type);
    } else 
    {
        throw EposException("Encoder type not supported");
    }
    const int resolution(encoder_nh.param("resolution", 0));
    const int gear_ratio(encoder_nh.param("gear_ratio", 0));
    if (resolution == 0 || gear_ratio == 0) {
        throw EposException("Parameters 'resolution' and 'gear_ratio' must be set");
    }
    const bool inverted_polarity(encoder_nh.param("inverted_polarity", false));
    VCS_NODE_COMMAND(SetIncEncoderParameter, epos_handle_, resolution, inverted_polarity);
    pulses_to_rad_ = 4.0 * resolution * gear_ratio / 2.0 / M_PI;

    // Enable the motor
    VCS_NODE_COMMAND_NO_ARGS(SetEnableState, epos_handle_);
}

EposDevice::~EposDevice()
{
    try {
        VCS_NODE_COMMAND_NO_ARGS(SetDisableState, epos_handle_);
    } catch (const EposException &e) {
        ROS_ERROR_STREAM(e.what());
    }
}

void EposDevice::clearDeviceErrors()
{
    unsigned char num_of_device_errors;
    // Get Current Error nums
    VCS_NODE_COMMAND(GetNbOfDeviceError, epos_handle_, &num_of_device_errors);
    for (int i = 1; i <= num_of_device_errors; i++) {
        unsigned int device_error_code;
        VCS_NODE_COMMAND(GetDeviceErrorCode, epos_handle_, i, &device_error_code);
        ROS_WARN_STREAM("EPOS Device Error: 0x" << std::hex << device_error_code);
    }
    // Clear Errors
    VCS_NODE_COMMAND_NO_ARGS(ClearFault, epos_handle_);
    // Check to make sure number of erros is zero after clearing
    VCS_NODE_COMMAND(GetNbOfDeviceError, epos_handle_, &num_of_device_errors);
    if (num_of_device_errors > 0) {
        throw EposException("Unable to clear " + std::to_string(num_of_device_errors) + " faults.");
    }
}

double EposDevice::readPosition()
{
    int raw_position;
    VCS_NODE_COMMAND(GetPositionIs, epos_handle_, &raw_position);
    return raw_position / pulses_to_rad_;
}

double EposDevice::readVelocity()
{
    int raw_velocity;
    VCS_NODE_COMMAND(GetPositionIs, epos_handle_, &raw_velocity);
    return raw_velocity * RPM_TO_RADS;
}

double EposDevice::readCurrent()
{
    short raw_current;
    VCS_NODE_COMMAND(GetCurrentIs, epos_handle_, &raw_current);
    return raw_current / 1000.0;
}

void EposDevice::writeCurrent(const double cmd)
{
    if (cmd == last_command_)
    {
        return;
    }
    last_command_ = cmd;
    long milliamps = static_cast<long>(cmd * 1000.0); // A to mA
    VCS_NODE_COMMAND(SetCurrentMustEx, epos_handle_, milliamps);
}

}
