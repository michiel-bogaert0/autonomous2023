#ifndef IOLOGIK_DRIVER_HPP
#define IOLOGIK_DRIVER_HPP
#include "managed_node.hpp"
#include <mxio.h>
#include <node_fixture/node_fixture.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
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
  ros::Publisher input8_pub_;

  ros::Subscriber output0_sub_;
  ros::Subscriber output1_sub_;
  ros::Subscriber output2_sub_;
  ros::Subscriber output3_sub_;
  ros::Subscriber output4_sub_;
  ros::Subscriber output5_sub_;

  const char *ip = "192.168.50.4";
  int port = 502;
  const int timeout = 2000;
  void output0Callback(std_msgs::Bool msg);
  void output1Callback(std_msgs::Bool msg);
  void output2Callback(std_msgs::Bool msg);
  void output3Callback(std_msgs::Bool msg);
  void output4Callback(std_msgs::Bool msg);
  void output5Callback(std_msgs::Bool msg);

  void CheckErr(int iHandle, int iRet,
                char *szFunctionName); // check function execution result

  bool output0 = 0;
  bool output1 = 0;
  bool output2 = 0;
  bool output3 = 0;
  bool output4 = 0;
  bool output5 = 0;

  bool o0_changed = true; // so that we don't keep writing the same value but
                          // only when the value is changed
  bool o1_changed = true;
  bool o2_changed = true;
  bool o3_changed = true;
  bool o4_changed = true;
  bool o5_changed = true;

  int iRet;    // return value from the CheckErr function
  int iHandle; // handle
};

#endif // iologik_HPP
