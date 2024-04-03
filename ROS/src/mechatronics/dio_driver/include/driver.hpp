#ifndef DIODRIVER_HPP
#define DIODRIVER_HPP

#include "VecowLinux.h"
#include "managed_node.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

#define _STDCALL_
#define LLIB dlsym

// Typedefs for libvecow stuff

typedef BOOL(_STDCALL_ *_B_IBIBIWIW)(BYTE *I1, BYTE *I2, WORD *I3,
                                     WORD *I4);     // return_dll_version
typedef BOOL(_STDCALL_ *_B_OBOB)(BYTE O1, BYTE O2); // initial_SIO
typedef BOOL(_STDCALL_ *_B_OBOB)(BYTE O1, BYTE O2); // initial_SIO
typedef BOOL(_STDCALL_ *_B_IBIBIBIW)(
    BYTE *I1, BYTE *I2, BYTE *I3,
    WORD *I4); // get_IO1_configuration get_IO2_configuration
typedef BOOL(_STDCALL_ *_B_OBOBOBOW)(
    BYTE O1, BYTE O2, BYTE O3,
    WORD O4); // set_IO1_configuration set_IO2_configuration
typedef BOOL(_STDCALL_ *_B_IBIB)(BYTE *I1, BYTE *I2); // Get_DIO1_ Get_DIO2_
typedef BOOL(_STDCALL_ *_B_OB)(BYTE O1);              // set_DIO1 set_DIO2
typedef BOOL(_STDCALL_ *_B_IW)(WORD *I1);             // get_GPIO1 get_GPIO2
typedef BOOL(_STDCALL_ *_B_OW)(WORD O1);              // set_GPIO1 set_GPIO2
typedef BOOL(_STDCALL_ *_B_IB)(BYTE *I1);             // get_CPU_temperature

class DIODriver : public node_fixture::ManagedNode {

public:
  explicit DIODriver(ros::NodeHandle &n, int bank_id);

  /**
   * @brief Configure the node
   *
   * This function is called by the ManagedNode base class to configure the
   * node. Basically reads parameters and makes publishers/subscribers
   */
  virtual void doConfigure();

  /**
   * @brief Cleanup the node
   *
   * This function is called by the ManagedNode base class to cleanup the node.
   * Basically removes publishers/subscribers
   */
  virtual void doCleanup();

  /**
   * @brief Activate the node
   *
   * This function is called by the ManagedNode base class to activate the node.
   * Basically reads inputs and writes outputs
   */
  virtual void active();

private:
  /**
   * @brief Check if there is an error
   *
   * This function checks if there is an error and prints a message if there is
   * Also sets the health to error in that case
   *
   * Returns true if there is an error, false otherwise
   */
  bool isError(bool ret_val, std::string msg);

  /**
   * @brief Set the output callback
   *
   * This function is called when a message is received on the "set output"
   * topics It sets the output value of the corresponding output "i"
   */
  void SetOutputCallback(const std_msgs::Bool::ConstPtr &msg, int i);

private:
  ros::NodeHandle nh;

  int bank_id;

  ros::Publisher cpu_temp_pub;
  bool enable_temp;

  ros::Publisher di_pub[8];  // iPC INPUTS (to be read)
  ros::Subscriber do_sub[8]; // iPC OUTPUTS (to be written)
  bool enabled_do[8];        // iPC OUTPUTS (to be written)
  bool enabled_di[8];        // iPC INPUTS (to be read)

  // Libvecow.so stuff

  void *lib_handle = NULL;
  _B_OBOB initialSIO = NULL;
  _B_IBIBIBIW getIOconfiguration = NULL;
  _B_OBOBOBOW setIOconfiguration = NULL;
  _B_IBIB getDIO = NULL;
  _B_OB setDIO = NULL;
  _B_IB getCPUtemperature = NULL;

  BYTE DIOISIO[2],  // DIOISIO [Group1 / Group2]
      DIONPN[2][2], // DIONPN  [Group1 / Group2] [ In / Out]
      DIO[2][2];    //    DIO  [Group1 / Group2] [ In / Out]
  WORD GPIO[2],     //   GPIO  [Group1 / Group2]
      DIOM[2];      //   DIOM  [Group1 / Group2]
};

#endif