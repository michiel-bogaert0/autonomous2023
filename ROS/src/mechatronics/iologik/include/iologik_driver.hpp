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
  ros::Publisher input0_pub_;
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
  const float max_value = 10.0f;
  const int timeout = 2000;
  void output0Callback(std_msgs::Float64 msg);
  void output1Callback(std_msgs::Float64 msg);
  void CheckErr(int iHandle, int iRet,
                char *szFunctionName);        // check function execution result
  void CheckInput(int channel, double input); // go into error when a current is
                                              // not within the correct range
  double output0 = 4;
  double output1 = 4;
  double minimum_output_current;
  double maximum_output_current;
  double minimum_input_current;
  double maximum_input_current;
  bool enable_i0;
  bool enable_i1;
  bool enable_i2;
  bool enable_i3;
  bool enable_i4;
  bool enable_i5;
  bool enable_i6;
  bool enable_i7;
  bool enable_o0;
  bool enable_o1;
  int iRet;    // return value
  int iHandle; // handle
};

#endif // iologik_driver_HPP
