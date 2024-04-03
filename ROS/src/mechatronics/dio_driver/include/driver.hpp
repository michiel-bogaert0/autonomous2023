#ifndef DIODRIVER_HPP
#define DIODRIVER_HPP

#include "VecowLinux.h"
#include "managed_node.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

#define _STDCALL_
#define LLIB dlsym

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

class DIODriver : public node_fixture::ManagedNode {

public:
  explicit DIODriver(ros::NodeHandle &n, int bank_id);

  virtual void doConfigure();
  virtual void doCleanup();
  virtual void active();

private:
  bool isError(bool ret_val, std::string msg);

  void SetOutputCallback(const std_msgs::Bool::ConstPtr &msg, int i);

  ros::NodeHandle nh;

  int bank_id;

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

  BYTE DIOISIO[2],  // DIOISIO [Group1 / Group2]
      DIONPN[2][2], // DIONPN  [Group1 / Group2] [ In / Out]
      DIO[2][2];    //    DIO  [Group1 / Group2] [ In / Out]
  WORD GPIO[2],     //   GPIO  [Group1 / Group2]
      DIOM[2];      //   DIOM  [Group1 / Group2]
};

#endif