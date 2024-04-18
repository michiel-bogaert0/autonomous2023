/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the Sim
           For a more detailed simulation example, see sim_hw_interface.h
*/

#ifndef GEN4_CONTROL__GEN4_HW_INTERFACE_H
#define GEN4_CONTROL__GEN4_HW_INTERFACE_H

#include <ugr_ros_control/generic_hw_interface.hpp>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <can_msgs/Frame.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ugr_msgs/State.h>
#include <ugr_msgs/CanFrame.h>
#include <ugr_msgs/KeyValueFloat.h>

namespace gen4_control
{
/// \brief Hardware interface for a robot
class Gen4HWInterface : public ugr_ros_control::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  explicit Gen4HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \brief Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

  void state_change(const ugr_msgs::State::ConstPtr& msg);

  void handle_vel_msg();
  void publish_steering_msg(float steering);
  void publish_vel_msg(float vel, int axis);
  void publish_torque_msg(float axis);
  void send_torque_on_can(float axis, int id);
  void can_callback_axis0(const std_msgs::Float32::ConstPtr& msg);
  void can_callback_axis1(const std_msgs::Float32::ConstPtr& msg);
  void can_callback_steering(const std_msgs::Float32::ConstPtr& msg);

  void yaw_rate_callback(const sensor_msgs::Imu::ConstPtr& msg);

  float torque_vectoring();

private:
  int drive_joint_id;
  int steering_joint_id;

  std::string axis_rear_frame;

  bool is_running = false;

  int IMU_ids[2] = { 0xE2, 0xE3 };

  ros::Publisher can_pub;
  ros::Publisher vel_pub;
  ros::Subscriber can_sub;
  ros::Subscriber state_sub;
  ros::Subscriber can_axis0_sub;
  ros::Subscriber can_axis1_sub;
  ros::Subscriber can_steering_sub;
  ros::Subscriber jaw_rate_sub;
  ros::Publisher vel_left_pub;
  ros::Publisher vel_right_pub;
  float steer_max_step;

  // harware parameters
  float n_polepairs;
  float wheel_diameter;
  float gear_ratio;

  // state variables
  float cur_velocity_axis0;
  float cur_velocity_axis1;
  float cur_steering;

  // PID for torque vectoring
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float prev_error;
  double prev_time;
  double this_time;
  float yaw_rate;

  // parameters for torque vectoring
  bool use_torque_vectoring;
  float max_dT;
  float l_wheelbase;
  float COG;
  float Cyf;
  float Cyr;
  float m;
  float lr;
  float lf;

};  // class

}  // namespace gen4_control

#endif
