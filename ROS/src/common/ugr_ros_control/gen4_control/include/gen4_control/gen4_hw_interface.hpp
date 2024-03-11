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

  void handle_vel_msg(const can_msgs::Frame::ConstPtr& msg, uint32_t axis_id);

  void publish_steering_msg(float steering);
  void publish_vel_msg(float vel, int axis);
  void can_callback(const can_msgs::Frame::ConstPtr& msg);

private:
  int drive_joint_id;
  int steering_joint_id;

  std::string axis0_frame;
  std::string axis1_frame;

  float wheel_diameter;
  float gear_ratio;

  bool is_running = false;

  int IMU_ids[2] = { 0xE2, 0xE3 };

  ros::Publisher can_pub;
  ros::Subscriber can_sub;
  ros::Subscriber state_sub;
  ros::Publisher vel_left_pub;
  ros::Publisher vel_right_pub;
  float steer_max_step;

  float cur_velocity_axis0;
  float cur_velocity_axis1;
  float cur_steering;

};  // class

}  // namespace gen4_control

#endif
