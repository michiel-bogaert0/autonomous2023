#ifndef IOLOGIK_DRIVER_HPP
#define IOLOGIK_DRIVER_HPP
#include "managed_node.hpp"
#include <mxio.h>
#include <node_fixture/node_fixture.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <vector>

class iologik : public node_fixture::ManagedNode {
public:
  ros::NodeHandle n_;
  explicit iologik(ros::NodeHandle &n);
  void open();
  void close();
  void active() override;
  void doConfigure() override;
  void doActivate() override;
  void doDeactivate() override;

private:
  ros::Publisher
      input0_pub_; // inputs need to be grouped to make sure
                   // the reading is efficient (otherwise it won't work)
  ros::Publisher input1_pub_;
  ros::Publisher input2_pub_;
  ros::Publisher input3_pub_;
  ros::Publisher input4_pub_;
  ros::Publisher input5_pub_;
  ros::Publisher input6_pub_;
  ros::Publisher input7_pub_;
  ros::Subscriber output0_sub_;
  ros::Subscriber output1_sub_;
  const char *ip = "192.168.50.3";
  int port = 502;
  const int timeout = 2000;
  void output0Callback(std_msgs::Float64 msg);
  void output1Callback(std_msgs::Float64 msg);
  void CheckErr(int iHandle, int iRet,
                char *szFunctionName); // check function execution result
  void CheckInput(int channel, double input);
  double output0;
  double output1;
  double minimum_output_current;
  double maximum_output_current;
  double minimum_input_current;
  double maximum_input_current;
  int start_channel = -1; // lowest input that is enabled
  int end_channel = -1;
  int enabled_channels = 0; // number of enabled channels, the enabled channels
                            // ALWAYS have to be grouped
  bool enable_inputs[8];
  bool enable_o0;
  bool enable_o1;
  bool o0_changed = false; // so that we don't keep writing the same value but
                           // only when the value is changed
  bool o1_changed = false;

  float scaling_inputs[8];

  int iRet;    // return value from the CheckErr function
  int iHandle; // handle
};

#endif // iologik_HPP
