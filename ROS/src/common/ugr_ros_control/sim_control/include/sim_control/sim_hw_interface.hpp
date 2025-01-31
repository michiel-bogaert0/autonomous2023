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

#ifndef SIM_CONTROL__SIM_HW_INTERFACE_H
#define SIM_CONTROL__SIM_HW_INTERFACE_H

#include <ugr_ros_control/generic_hw_interface.hpp>
#include <sim_control/bicycle_model.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/service_client.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/Float32.h>

namespace sim_control
{
/// \brief Hardware interface for a robot
class SimHWInterface : public ugr_ros_control::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  explicit SimHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \brief Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

  void publish_gt(const ros::TimerEvent&);
  void apply_noise_and_quantise(float& x, std::vector<double> noise);
  void publish_encoder(const ros::TimerEvent&);
  void publish_imu(const ros::TimerEvent&);

  float torque_vectoring(float axis0, float delta);

private:
  BicycleModel* model;
  int drive_joint_id;
  int steering_joint_id;

  ros::Publisher gt_pub, encoder_pub, imu_pub, TV_axis0, TV_axis1;
  tf2_ros::TransformBroadcaster br;
  std::string world_frame, gt_base_link_frame, base_link_frame;
  std::vector<double> encoder_noise, imu_angular_velocity_noise, imu_acceleration_noise;

  ros::Timer gt_timer, imu_timer, encoder_timer;

  // PID for torque vectoring
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float prev_error;
  float prev_time;
  float this_time;
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

}  // namespace sim_control

#endif
