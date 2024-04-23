#include "driver.hpp"
#include <filesystem>
#include <iostream>
namespace fs = std::filesystem;

DIODriver::DIODriver(ros::NodeHandle &n) : ManagedNode(n, "dio_driver") {
  this->nh = n;

  // Intialize libvecow.so
  std::string current_path = fs::current_path().u8string();
  std::string library_path = current_path + "/libvecow.so";
  void *lib_handle = dlopen(library_path.c_str(), RTLD_NOW | RTLD_GLOBAL);

  if (lib_handle != nullptr) {
    ROS_INFO("libvecow.so loaded successfully\n");
  } else {
    ROS_ERROR("libvecow.so failed to load: %s", dlerror());
    return;
  }

  // Get functions
  returndllversion = (_B_IBIBIWIW)LLIB(lib_handle, "return_dll_version");
  initialSIO = (_B_OBOB)LLIB(lib_handle, "initial_SIO");
  getIOconfiguration[0] =
      (_B_IBIBIBIW)LLIB(lib_handle, "get_IO1_configuration");
  setIOconfiguration[0] =
      (_B_OBOBOBOW)LLIB(lib_handle, "set_IO1_configuration");
  getDIO[0] = (_B_IBIB)LLIB(lib_handle, "get_DIO1");
  setDIO[0] = (_B_OB)LLIB(lib_handle, "set_DIO1");

  getIOconfiguration[1] =
      (_B_IBIBIBIW)LLIB(lib_handle, "get_IO2_configuration");
  setIOconfiguration[1] =
      (_B_OBOBOBOW)LLIB(lib_handle, "set_IO2_configuration");
  getDIO[1] = (_B_IBIB)LLIB(lib_handle, "get_DIO2");
  setDIO[1] = (_B_OB)LLIB(lib_handle, "set_DIO2");

  getECbaseaddress = (_B_NE)LLIB(lib_handle, "get_EC_base_address");

  getCPUtemperature = (_B_IB)LLIB(lib_handle, "get_CPU_temperature");

  // Getting privileged access to IO
  iopl(3);

  // Initialize IO as isolated sources (not sure if needed)
  if (!initialSIO(1, 0)) {
    ROS_ERROR("Failed to initialize SIO");
    exit(-1);
  }

  // Set IO configuration (otherwise DO doesn't work)
  if (!setIOconfiguration[0](1, 0, 1, 0xFF00) ||
      !setIOconfiguration[1](1, 0, 1, 0xFF00)) {
    ROS_ERROR("Failed to set IO config!");
    exit(-1);
  }
}

void DIODriver::doCleanup() {
  // Remove pubs and subs
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < 8; i++) {
      if (this->enabled_do[j][i]) {
        this->do_sub[j][i].shutdown();
      }

      if (this->enabled_di[j][i]) {
        this->di_pub[j][i].shutdown();
      }
    }
  }

  this->setHealthStatus(0, "OK", {});
}

void DIODriver::active() {

  for (int j = 0; j < 2; j++) {
    // Read inputs
    BYTE statusDO, statusDI;

    if (this->isError(this->getDIO[j](&statusDO, &statusDI),
                      "Failed to get DIO status"))
      return;

    // Publish results
    for (int i = 0; i < 8; i++) {
      if (this->enabled_di[j][i]) {
        std_msgs::Bool msg;
        msg.data = (statusDI & (1 << i)) != 0;
        this->di_pub[j][i].publish(msg);
      }
    }
  }

  if (this->enable_temp) {

    BYTE CPUTemp = 0;
    if (this->isError(this->getCPUtemperature(&CPUTemp),
                      "Failed to get CPU temperature"))
      return;

    std_msgs::UInt8 msg;
    msg.data = CPUTemp;
    this->cpu_temp_pub.publish(msg);
  }
}

void DIODriver::doConfigure() {

  // Get which io are enabled and make correct subs and pubs
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < 8; i++) {
      this->enabled_do[j][i] = this->nh.param<bool>(
          "bank" + std::to_string(j + 1) + "/enable_DO" + std::to_string(i),
          false);
      this->enabled_di[j][i] = this->nh.param<bool>(
          "bank" + std::to_string(j + 1) + "/enable_DI" + std::to_string(i),
          false);

      if (this->enabled_do[j][i]) {
        this->do_sub[j][i] = this->nh.subscribe<std_msgs::Bool>(
            "bank" + std::to_string(j + 1) + "/DO" + std::to_string(i), 1,
            boost::bind(&DIODriver::SetOutputCallback, this, _1, i, j));
      }

      if (this->enabled_di[j][i]) {
        this->di_pub[j][i] = this->nh.advertise<std_msgs::Bool>(
            "bank" + std::to_string(j + 1) + "/DI" + std::to_string(i), 1);
      }
    }
  }

  // Other pubs
  this->enable_temp = this->nh.param<bool>("enable_temp", false);
  if (this->enable_temp) {
    this->cpu_temp_pub = this->nh.advertise<std_msgs::UInt8>("cpu_temp", 1);
  }

  this->setHealthStatus(0, "OK", {});
}

bool DIODriver::isError(bool ret_val, std::string msg) {
  if (!ret_val) {
    ROS_ERROR(msg.c_str());
    this->setHealthStatus(2, msg, {});
    return true;
  }
  return false;
}

void DIODriver::SetOutputCallback(const std_msgs::Bool::ConstPtr &msg, int i,
                                  int j) {

  // First get current status
  BYTE statusDO, statusDI;

  if (this->isError(this->getDIO[j](&statusDO, &statusDI),
                    "Failed to get DIO status"))
    return;

  // Set the bit
  if (msg->data) {
    statusDO |= 1 << i;
  } else {
    statusDO &= ~(1 << i);
  }

  // Set the new status
  if (this->isError(this->setDIO[j](statusDO), "Failed to set DIO status"))
    return;
}