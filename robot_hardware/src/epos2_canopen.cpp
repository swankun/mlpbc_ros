#include <robot_hardware/epos2_canopen.h>

namespace robot_hardware
{

void
Epos2Driver::OnBoot(canopen::NmtState /*st*/, char es,
                    const std::string& what) 
{
    if (!es || es == 'L') {
      std::cout << "slave " << static_cast<int>(id()) << " booted sucessfully"
                << std::endl;
    } else {
      std::cout << "slave " << static_cast<int>(id())
                << " failed to boot: " << what << std::endl;
    }
}


void
Epos2Driver::OnConfig(std::function<void(std::error_code ec)> res) 
{
    
}

void
Epos2Driver::OnDeconfig(std::function<void(std::error_code ec)> res)
{

}


void
Epos2Driver::OnRpdoWrite(uint16_t idx, uint8_t subidx)
{

}


Epos2CurrentMode::Epos2CurrentMode() : 
    poll_(ctx_),
    loop_(poll_.get_poll()),
    exec_(loop_.get_executor()),
    timer_(poll_, exec_, CLOCK_MONOTONIC),
{
    Epos2Driver drier(exec_, master_, 1);

}


} // namespace robot_hardware
