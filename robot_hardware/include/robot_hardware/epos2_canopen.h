#ifndef ROBOT_HARDWARE_EPOS2_CANOPEN_H
#define ROBOT_HARDWARE_EPOS2_CANOPEN_H

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>

#include <iostream>

namespace robot_hardware
{ 

using namespace std::chrono_literals;
using namespace lely;

class Epos2Driver : public canopen::FiberDriver
{
    public:
        using FiberDriver::FiberDriver;
        Epos2Driver() = default;

    private:
        /**
         * This function gets called when the boot-up process of the slave
         * completes. The 'st' parameter contains the last known NMT state of
         * the slave (typically pre-operational), 'es' the error code (0 on
         * success), and 'what' a description of the error, if any.
         */
        void
        OnBoot(canopen::NmtState /*st*/, char es,
               const std::string& what) noexcept override;

        /**
         * This function gets called during the boot-up process for the slave.
         * The 'res' parameter is the function that MUST be invoked when the
         * configuration is complete. Because this function runs as a task
         * inside a coroutine, it can suspend itself and wait for an
         * asynchronous function, such as an SDO request, to complete.
         */
        void
        OnConfig(std::function<void(std::error_code ec)> res) noexcept override;

        /**
         * This function is similar to OnConfg(), but it gets called by the
         * AsyncDeconfig() method of the master.
         */
        void
        OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override;

        /**
         * This function gets called every time a value is written to the local object
         * dictionary of the master by an RPDO (or SDO, but that is unlikely for a
         * master), *and* the object has a known mapping to an object on the slave for
         * which this class is the driver. The 'idx' and 'subidx' parameters are the
         * object index and sub-index of the object on the slave, not the local object
         * dictionary of the master.
         */
        void
        OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;

}; // class Epos2Driver

class Epos2CurrentMode
{
    public:
        Epos2CurrentMode();
        ~Epos2CurrentMode();

        void setCurrent(const double milliamps);
        void getVelocity(double &vel);
        void getPosition(double &vel);
        void getCurrent(double &vel);
        
    
    private:
        Epos2Driver driver_;
        // Create an I/O context to synchronize I/O services during shutdown.
        io::Context ctx_;
        // Create an platform-specific I/O polling instance to monitor the CAN bus, as
        // well as timers and signals.
        io::Poll poll_;
        // Create a polling event loop and pass it the platform-independent polling
        // interface. If no tasks are pending, the event loop will poll for I/O
        // events.
        ev::Loop loop_;
        // I/O devices only need access to the executor interface of the event loop.
        ev::Executor exec_;
        // Create a timer using a monotonic clock, i.e., a clock that is not affected
        // by discontinuous jumps in the system time.
        io::Timer timer_;
        // Create a virtual SocketCAN CAN controller and channel, and do not modify
        // the current CAN bus state or bitrate.
        io::CanController ctrl_;
        io::CanChannel chan_;
        // Create a CANopen master. The master is asynchronous, which means
        // every user-defined callback for a CANopen event will be posted as a
        // task on the event loop, instead of being invoked during the event
        // processing by the stack.
        canopen::AsyncMaster master_;
        // Create a signal handler.
        io::SignalSet sigset_;
};

} // namespace epos2_canopen

#endif // ROBOT_HARDWARE_EPOS2_CANOPEN_H
