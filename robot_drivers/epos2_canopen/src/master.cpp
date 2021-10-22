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
    // loop_.stop();
    ctx_.shutdown();
    });
  });
}

void CanopenMaster::startDevice()
{
  pDriver_ = std::make_unique<Epos2Driver>(exec_, master_, slave_id_);
  master_.Reset();
  // std::cout << "============================================== " << std::endl;
  // std::cout << "in startDevice(), created pDriver at " << &pDriver_ << std::endl;
  // std::cout << "in startDevice(), testval_ is at " << &pDriver_->testval_ << std::endl;
  // std::cout << "============================================== " << std::endl;
  // ev_fiber_exec_init(exec_, ev_fiber_exec_get_inner_exec(exec_));
  loop_.run();
  pDriver_.reset(nullptr);
  std::cout << "Loop stopped" << std::endl;
}

void CanopenMaster::stopDevice()
{
  if (isShutdown())
    return;
  // Disable the heartbeat consumer on the master.
  // Disable the heartbeat producer on the slave.
  pDriver_->ConfigHeartbeat(0ms);
  lely::canopen::SdoFuture<void> fut = master_.AsyncWrite<uint16_t>(exec_, slave_id_, 0x1017, 0, 0, 2000ms);
  while (!fut.is_ready())
  {
    // master_.AsyncWrite<uint16_t>(exec_, slave_id_, 0x1017, 0, 0, 1000ms);
    master_.AsyncWait(100ms);
  }
  // master_.Write<uint16_t>(0x1017,0,0);
  // pDriver_->Wait(AsyncWrite<uint16_t>(0x1017, 0, 0));
  // Disable the heartbeat consumer on the slave.
  fut = master_.AsyncWrite<uint32_t>(exec_, slave_id_, 0x1016, 1, 0, 2000ms);
  while (!fut.is_ready())
  {
    master_.AsyncWait(100ms);
  }
  // master_.Write<uint32_t>(0x1016,1,0);
  // pDriver_->Wait(AsyncWrite<uint32_t>(0x1016, 1, 0));
  // loop_.kill(loop_.self());
  loop_.stop();
  // ctx_.shutdown();
  // exec_.on_task_fini();
  // exec_.on_task_fini();
  // exec_.on_task_fini();
  // ev_fiber_exec_destroy(pDriver_->GetExecutor());
  // ev_fiber_exec_fini(ev_fiber_exec_get_inner_exec(exec_));
  // ev_exec_on_task_fini(exec_);
  // ev_exec_on_task_fini(exec_);
}

bool CanopenMaster::isShutdown()
{
  return !master_.IsReady(slave_id_);
}

void CanopenMaster::setCurrent(const double milliamps)
{
  pDriver_->tpdo_mapped[0x2030][0] = static_cast<int16_t>(750);
}

void CanopenMaster::getVelocity(double &vel)
{
  vel = static_cast<double>(pDriver_->testval_);
}

void CanopenMaster::getPosition(double &vel)
{}

void CanopenMaster::getCurrent(double &vel)
{}

} // namespace robot_hardware
