#include<epos2_canopen/master.h>

namespace epos2_canopen
{

CanopenMaster::CanopenMaster(std::string &device, std::string &eds_path, 
                             uint8_t master_nid, uint8_t slave_nid) 
  : master_id_(master_nid)
  , slave_id_(slave_nid)
  , poll_(ctx_)
  , loop_(poll_.get_poll())
  , exec_(loop_.get_executor())
  , timer_(poll_, exec_, CLOCK_MONOTONIC)
  , ctrl_(device.c_str())
  , chan_(poll_, exec_)
  , master_(timer_, chan_, eds_path, "", master_nid)
  , sigset_(poll_, exec_)
{
  chan_.open(ctrl_);
  sigset_.insert(SIGHUP);
  sigset_.insert(SIGINT);
  sigset_.insert(SIGTERM);
  // Submit a task to be executed when a signal is raised. We don't care which.
  sigset_.submit_wait([&](int /*signo*/) {
    // If the signal is raised again, terminate immediately.
    sigset_.clear();
    // Tell the master to start the deconfiguration process for all nodes, and
    // submit a task to be executed once that process completes.
    master_.AsyncDeconfig().submit(exec_, [&]() {
    // Perform a clean shutdown.
    ctx_.shutdown();
    });
  });
}

void CanopenMaster::startDevice()
{
  pDriver_ = std::make_unique<Epos2Driver>(exec_, master_, slave_id_);
  // std::cout << "============================================== " << std::endl;
  // std::cout << "in startDevice(), created pDriver at " << &pDriver_ << std::endl;
  // std::cout << "in startDevice(), testval_ is at " << &pDriver_->testval_ << std::endl;
  // std::cout << "============================================== " << std::endl;
  master_.Reset();
  loop_.run();
}

void CanopenMaster::setCurrent(const double milliamps)
{}

void CanopenMaster::getVelocity(double &vel)
{
  vel = static_cast<double>(pDriver_->testval_);
}

void CanopenMaster::getPosition(double &vel)
{}

void CanopenMaster::getCurrent(double &vel)
{}

} // namespace robot_hardware
