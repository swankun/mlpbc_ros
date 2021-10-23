#include<epos2_canopen/master.h>

namespace epos2_canopen
{

CanopenMaster::CanopenMaster(const std::string &device, 
                             const std::string &eds_path,
                             const std::string &eds_bin_path,
                             uint8_t master_nid, uint8_t slave_nid) 
  : master_id_(master_nid)
  , slave_id_(slave_nid)
  , poll_(ctx_)
  , loop_(poll_.get_poll())
  , exec_(loop_.get_executor())
  , timer_(poll_, exec_, CLOCK_MONOTONIC)
  , ctrl_(device.c_str())
  , chan_(poll_, exec_)
  , master_(timer_, chan_, eds_path, eds_bin_path, master_nid)
  , sigset_(poll_, exec_)
{ init(); }

CanopenMaster::CanopenMaster(const std::string &device, 
                             const std::string &eds_path,
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
{ init(); }


void CanopenMaster::init()
{
  chan_.open(ctrl_);
  // sigset_.insert(SIGHUP);
  // sigset_.insert(SIGINT);
  // sigset_.insert(SIGTERM);
  // // Submit a task to be executed when a signal is raised. We don't care which.
  // sigset_.submit_wait([&](int /*signo*/) {
  //   // If the signal is raised again, terminate immediately.
  //   sigset_.clear();
  //   // Tell the master to start the deconfiguration process for all nodes, and
  //   // submit a task to be executed once that process completes.
  //   master_.AsyncDeconfig().submit(exec_, [&]() {
  //   // Perform a clean shutdown.
  //   ctx_.shutdown();
  //   });
  // });
}

void CanopenMaster::startDevice()
{
  pDriver_ = std::make_unique<Epos2Driver>(exec_, master_, slave_id_);
  master_.Reset();
  loop_.run();
  pDriver_.reset(nullptr);
  std::cout << "Loop stopped" << std::endl;
}

void CanopenMaster::stopDevice()
{
  if (isShutdown()) { return; }
  lely::canopen::SdoFuture<void> sdo_fut;
  lely::ev::Future<size_t,void> deconfig_fut = master_.AsyncDeconfig();
  bool success = false;
  for (size_t i=0; i<20; i++)
  {    
    if (deconfig_fut.is_ready()) {
      success = true;
      break;
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
  }
  if (!success) // brute-force shutdown
  {
    std::cout << "Graceful shutdown failed! Begin brute-force shutdown." << std::endl;

    // Disable hearbeat consumer on master
    pDriver_->ConfigHeartbeat(0ms);

    // Disable operation on the slave.
    sdo_fut = master_.AsyncWrite<uint16_t>(exec_, slave_id_, 0x6040, 0, 0x06, 2000ms);
    wait_for(sdo_fut, 2000);

    // Disable the heartbeat producer on the slave.
    sdo_fut = master_.AsyncWrite<uint16_t>(exec_, slave_id_, 0x1017, 0, 0, 2000ms);
    wait_for(sdo_fut, 2000);

    // Disable the heartbeat consumer on the slave.
    sdo_fut = master_.AsyncWrite<uint32_t>(exec_, slave_id_, 0x1016, 1, 0, 2000ms);
    wait_for(sdo_fut, 2000);
  }
  loop_.stop();
}

bool CanopenMaster::isShutdown()
{
  return !master_.IsReady(slave_id_);
}

void CanopenMaster::setCurrent(const double milliamps)
{
  pDriver_->tpdo_mapped[0x2030][0] = static_cast<int16_t>(milliamps);
  // double val;
  // getPosition(val);
  // pDriver_->tpdo_mapped[0x4000][0] = static_cast<uint32_t>(++val);
}

void CanopenMaster::getVelocity(double &val)
{
  int32_t raw = pDriver_->rpdo_mapped[0x606C][0];
  // uint32_t raw = pDriver_->rpdo_mapped[0x4001][0];
  val = static_cast<double>(raw);
}

void CanopenMaster::getPosition(double &val)
{
  int32_t raw = pDriver_->rpdo_mapped[0x6064][0];
  // uint32_t raw = pDriver_->rpdo_mapped[0x4001][0];
  val = static_cast<double>(raw);
}

void CanopenMaster::getCurrent(double &val)
{
  int16_t raw = pDriver_->rpdo_mapped[0x2027][0];
  // uint32_t raw = pDriver_->rpdo_mapped[0x4001][0];
  val = static_cast<double>(raw);
}

bool CanopenMaster::wait_for(lely::canopen::SdoFuture<void> fut, const int timeout_ms)
{
  size_t n = std::max(1, timeout_ms / 100);
  for (size_t i=0; i<n; i++)
  {    
    if (fut.is_ready()) break;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
  }
}

} // namespace robot_hardware
