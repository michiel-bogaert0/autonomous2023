#include "driver.hpp"

DIODriver::DIODriver(ros::NodeHandle &n, int bank_id)
    : ManagedNode(n, "dio_driver") {
  this->nh = n;
  this->bank_id = bank_id;

  // Intialize libvecow.so
  if ((this->lib_handle = dlopen("./libvecow.so", RTLD_NOW | RTLD_GLOBAL)) !=
      NULL) {
    ROS_INFO("libvecow.so loaded successfully");
  } else {
    ROS_ERROR("libvecow.so failed to load");
    dlclose(this->lib_handle);
    exit(-1);
  }

  // Get functions
  initialSIO = (_B_OBOB)LLIB(lib_handle, "initial_SIO");
  getIOconfiguration = (_B_IBIBIBIW)LLIB(
      lib_handle,
      ("get_IO" + std::to_string(bank_id) + "_configuration").c_str());
  setIOconfiguration = (_B_OBOBOBOW)LLIB(
      lib_handle,
      ("set_IO" + std::to_string(bank_id) + "_configuration").c_str());
  getDIO =
      (_B_IBIB)LLIB(lib_handle, ("get_DIO" + std::to_string(bank_id)).c_str());
  setDIO =
      (_B_OB)LLIB(lib_handle, ("set_DIO" + std::to_string(bank_id)).c_str());

  getCPUtemperature = (_B_IB)LLIB(lib_handle, "get_CPU_temperature");

  // Initialize IO as isolated sources (not sure if needed)
  if (!initialSIO(1, 0)) {
    ROS_ERROR("Failed to initialize SIO");
    exit(-1);
  }
}

void DIODriver::doCleanup() {
  // Remove pubs and subs
  for (int i = 0; i < 8; i++) {
    if (this->enabled_do[i]) {
      this->do_sub[i].shutdown();
    }

    if (this->enabled_di[i]) {
      this->di_pub[i].shutdown();
    }
  }
}

void DIODriver::active() {

  // Read inputs
  BYTE statusDO, statusDI;

  if (this->isError(this->getDIO(&statusDO, &statusDI),
                    "Failed to get DIO status"))
    return;

  // Publish results
  for (int i = 0; i < 8; i++) {
    if (this->enabled_di[i]) {
      std_msgs::Bool msg;
      msg.data = (statusDI & (1 << i)) != 0;
      this->di_pub[i].publish(msg);
    }
  }

  if (this->enable_temp) {

    BYTE CPUTemp = 0;
    if (this->isError(this->getCPUtemperature(&CPUTemp),
                      "Failed to get CPU temperature"))
      return;

    std_msgs::Int8 msg;
    msg.data = CPUTemp;
    this->cpu_temp_pub.publish(msg);
  }
}

void DIODriver::doConfigure() {

  // Get which io are enabled and make correct subs and pubs
  for (int i = 0; i < 8; i++) {
    this->enabled_do[i] =
        this->nh.param<bool>("enable_DO" + std::to_string(i), false);
    this->enabled_di[i] =
        this->nh.param<bool>("enable_DI" + std::to_string(i), false);

    if (this->enabled_do[i]) {
      this->do_sub[i] = this->nh.subscribe<std_msgs::Bool>(
          "DO" + std::to_string(i), 1,
          boost::bind(&DIODriver::SetOutputCallback, this, _1, i));
    }

    if (this->enabled_di[i]) {
      this->di_pub[i] =
          this->nh.advertise<std_msgs::Bool>("DI" + std::to_string(i), 1);
    }
  }

  // Other pubs
  this->enable_temp = this->nh.param<bool>("enable_temp", false);
  if (this->enable_temp) {
    this->cpu_temp_pub = this->nh.advertise<std_msgs::Int8>("cpu_temp", 1);
  }
}

bool DIODriver::isError(bool ret_val, std::string msg) {
  if (!ret_val) {
    ROS_ERROR(msg.c_str());
    this->setHealthStatus(2, msg, {});
    return false;
  }
  return true;
}

void DIODriver::SetOutputCallback(const std_msgs::Bool::ConstPtr &msg, int i) {

  // First get current status
  BYTE statusDO, statusDI;

  if (this->isError(this->getDIO(&statusDO, &statusDI),
                    "Failed to get DIO status"))
    return;

  // Set the bit
  if (msg->data) {
    statusDO |= 1 << i;
  } else {
    statusDO &= ~(1 << i);
  }

  // Set the new status
  if (this->isError(this->setDIO(statusDO), "Failed to set DIO status"))
    return;
}