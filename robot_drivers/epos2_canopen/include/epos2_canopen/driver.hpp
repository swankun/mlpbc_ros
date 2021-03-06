#ifndef EPOS2_CANOPEN_H
#define EPOS2_CANOPEN_H

#include <lely/coapp/fiber_driver.hpp>
#include <iostream>

namespace epos2_canopen
{ 

using namespace std::chrono_literals;
using namespace lely;

class Epos2Driver : public canopen::FiberDriver
{
    public:
        using FiberDriver::FiberDriver;
        uint32_t testval_ = 0;
        uint32_t getValue() {
            return testval_;
        };

    private:
        /**
         * This function gets called when the boot-up process of the slave
         * completes. The 'st' parameter contains the last known NMT state of
         * the slave (typically pre-operational), 'es' the error code (0 on
         * success), and 'what' a description of the error, if any.
         */
        void
        OnBoot(canopen::NmtState /*st*/, char es,
               const std::string& what) noexcept override
        {
            if (!es || es == 'L') {
            std::cout << "slave " << static_cast<int>(id()) << " booted sucessfully"
                        << std::endl;
            } else {
            std::cout << "slave " << static_cast<int>(id())
                        << " failed to boot: " << what << std::endl;
            }
        }

        /**
         * This function gets called during the boot-up process for the slave.
         * The 'res' parameter is the function that MUST be invoked when the
         * configuration is complete. Because this function runs as a task
         * inside a coroutine, it can suspend itself and wait for an
         * asynchronous function, such as an SDO request, to complete.
         */
        void
        OnConfig(std::function<void(std::error_code ec)> res) noexcept override 
        {
            try {
            // Perform a few SDO write requests to configure the slave. The
            // AsyncWrite() function returns a future which becomes ready once the
            // request completes, and the Wait() function suspends the coroutine for
            // this task until the future is ready.

            // Configure the slave to monitor the heartbeat of the master (node-ID 1)
            // with a timeout of 2000 ms.
            Wait(AsyncWrite<uint32_t>(0x1016, 1, (this->master.id() << 16) | 2000));
            // Configure the slave to produce a heartbeat every 1000 ms.
            Wait(AsyncWrite<uint16_t>(0x1017, 0, 1000));
            // Configure the heartbeat consumer on the master.
            ConfigHeartbeat(2000ms);

            // Reset object 4000:00 and 4001:00 on the slave to 0.
            // Wait(AsyncWrite<uint32_t>(0x4000, 0, 0));
            Wait(AsyncWrite<int16_t>(0x2030, 0, 0));
            Wait(AsyncWrite<int8_t>(0x6060, 0, -3));
            Wait(AsyncWrite<uint16_t>(0x6040, 0, 0x80));    // clear errors
            // Wait(AsyncWrite<uint16_t>(0x6040, 0, 0x06));    // Shutdown 
            // Wait(AsyncWrite<uint16_t>(0x6040, 0, 0x0f));    // Switch on & enable operation

            // Report success (empty error code).
            res({});
            } catch (canopen::SdoError& e) {
            // If one of the SDO requests resulted in an error, abort the
            // configuration and report the error code.
            res(e.code());
            }
        };

        /**
         * This function is similar to OnConfg(), but it gets called by the
         * AsyncDeconfig() method of the master.
         */
        void
        OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override 
        {
            std::cout << "==========================" << std::endl;
            std::cout << "Attempting to Deconfig...." << std::endl;
            std::cout << "==========================" << std::endl;
            try {
            Wait(AsyncWrite<int16_t>(0x2030, 0, 0));
            Wait(AsyncWrite<uint16_t>(0x6040, 0, 0x06));
            // Disable the heartbeat consumer on the master.
            ConfigHeartbeat(0ms);
            // Disable the heartbeat producer on the slave.
            Wait(AsyncWrite<uint16_t>(0x1017, 0, 0));
            // Disable the heartbeat consumer on the slave.
            Wait(AsyncWrite<uint32_t>(0x1016, 1, 0));
            res({});
            } catch (canopen::SdoError& e) {
            res(e.code());
            }
        };

        /**
         * This function gets called every time a value is written to the local object
         * dictionary of the master by an RPDO (or SDO, but that is unlikely for a
         * master), *and* the object has a known mapping to an object on the slave for
         * which this class is the driver. The 'idx' and 'subidx' parameters are the
         * object index and sub-index of the object on the slave, not the local object
         * dictionary of the master.
         */
        void
        OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override 
        {
            if (idx == 0x6041 && subidx == 0) {
                uint16_t status_word = rpdo_mapped[0x6041][0];
            } else if (idx == 0x6078 && subidx == 0) {
                int16_t current_actual = rpdo_mapped[0x6078][0];
            } else if (idx == 0x6061 && subidx == 0) {
                int8_t mode_of_operation = rpdo_mapped[0x6061][0];
            } else if (idx == 0x6064 && subidx == 0) {
                int32_t position_actual_value = rpdo_mapped[0x6064][0];
            } else if (idx == 0x606C && subidx == 0) {
                int32_t velocity_actual_value = rpdo_mapped[0x606C][0];
            } 
        };

}; // class Epos2Driver


} // namespace epos2_canopen

#endif // EPOS2_CANOPEN_H
