#ifndef EPOS2_CANOPEN_MASTER_H
#define EPOS2_CANOPEN_MASTER_H

#include <boost/thread.hpp>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/master.hpp>

#include <epos2_canopen/driver.hpp>

namespace epos2_canopen
{
class CanopenMaster
{
    public:
        CanopenMaster(const std::string &device, const std::string &eds_path,
                      uint8_t master_nid=255, uint8_t slave_nid=1);
        CanopenMaster(const std::string &device, const std::string &eds_path, 
                      const std::string &eds_bin_path, uint8_t master_nid=255, uint8_t slave_nid=1);

        void startDevice();
        void stopDevice();
        bool isShutdown();
        void setCurrent(const double milliamps);
        void getVelocity(double &val);
        void getPosition(double &val);
        void getCurrent(double &val);
        void clearFaults();
        void enableDevice();
        void disableOperation();
    
    private:
        void init();
        static bool wait_for(lely::canopen::SdoFuture<void> fut, const int timeout_ms);
        uint8_t master_id_, slave_id_;
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
        // Epos2Driver <: FiberDriver
        std::unique_ptr<Epos2Driver> pDriver_;
};
}   // namespace

#endif // EPOS2_CANOPEN_MASTER_H