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
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <gen4_control/gen4_hw_interface.hpp>
#include <random>
#include <tuple>
#include <math.h>

namespace gen4_control
{
Gen4HWInterface::Gen4HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ugr_ros_control::GenericHWInterface(nh, urdf_model)
{
  ros::NodeHandle n("~");

  ROS_INFO_NAMED("gen4_hw_interface", "Gen4HWInterface Ready.");
}

void Gen4HWInterface::init()
{
  // First do parent init
  ugr_ros_control::GenericHWInterface::init();

  ros::NodeHandle nh("~");

  nh.param("steer_max_step", steer_max_step, 1600.0f);

  this->can_pub = nh.advertise<can_msgs::Frame>("ugr/send_can", 10);
  this->can_axis0_sub =
      nh.subscribe<std_msgs::Float32>("/processed/Actual_ERPM", 1, &Gen4HWInterface::can_callback_axis0, this);
  this->can_axis1_sub =
      nh.subscribe<std_msgs::Float32>("/processed/Actual_ERPM", 1, &Gen4HWInterface::can_callback_axis1, this);
  this->can_steering_sub =
      nh.subscribe<std_msgs::Float32>("/processed/steering", 1, &Gen4HWInterface::can_callback_steering, this);

  this->jaw_rate_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, &Gen4HWInterface::yaw_rate_callback, this);

  this->state_sub = nh.subscribe<ugr_msgs::State>("/state", 1, &Gen4HWInterface::state_change, this);

  // Now check if configured joints are actually there. Also remember joint id
  std::string drive_joint0_name = nh_.param<std::string>("hardware_interface/drive_joint", "axis0");
  std::string drive_joint1_name = nh_.param<std::string>("hardware_interface/drive_joint", "axis1");
  std::string steering_joint_name = nh_.param<std::string>("hardware_interface/steering_joint", "axis_steering");

  this->axis0_frame = nh.param("axis0/frame", std::string("ugr/car_base_link/axis0"));
  this->axis1_frame = nh.param("axis1/frame", std::string("ugr/car_base_link/axis1"));
  this->wheel_diameter = nh.param("wheel_diameter", 16.0 * 2.54 / 100.0);  // in m
  this->gear_ratio = nh.param("gear_ratio", 3.405);

  drive_joint0_id = std::find(joint_names_.begin(), joint_names_.end(), drive_joint0_name) - joint_names_.begin();
  drive_joint1_id = std::find(joint_names_.begin(), joint_names_.end(), drive_joint1_name) - joint_names_.begin();
  steering_joint_id = std::find(joint_names_.begin(), joint_names_.end(), steering_joint_name) - joint_names_.begin();

  ROS_INFO_STREAM("Drive joint id: " << drive_joint0_id << "Drive joint1 id: " << drive_joint1_id
                                     << "< Steering joint id: " << steering_joint_id);

  if (drive_joint0_id >= joint_names_.size())
  {
    ROS_ERROR("Error: the parameter 'hardware_interface/joints_config/drive0_joint' must be given");
    throw std::invalid_argument("hardware_interface/joints_config/drive0_joint must be given");
  }

  if (drive_joint1_id >= joint_names_.size())
  {
    ROS_ERROR("Error: the parameter 'hardware_interface/joints_config/drive1_joint' must be given");
    throw std::invalid_argument("hardware_interface/joints_config/drive_joint1 must be given");
  }

  if (steering_joint_id >= joint_names_.size())
  {
    ROS_ERROR("Error: the parameter 'hardware_interface/joints_config/steering_joint' must be given");
    throw std::invalid_argument("hardware_interface/joints_config/steering_joint must be given");
  }

  ROS_INFO_NAMED("gen4_hw_interface", "Gen4HWInterface init'd.");

  // PID for Torque vectoring
  this->Kp = nh.param("Kp", 1);
  this->Ki = nh.param("Ki", 0.0);
  this->Kd = nh.param("Kd", 0.0);
  this->integral = 0.0;
  this->prev_error = 0.0;
  this->prev_time = ros::Time::now().toSec();
  this->yaw_rate = 0.0;

  // parametes for torque vectoring
  this->use_torque_vectoring = nh.param("use_torque_vectoring", false);
  this->max_dT = nh.param("max_dT", 4.0);
  this->l_wheelbase = nh.param("l_wheelbase", 1.518);
  this->COG = nh.param("COG", 0.5);
  this->Cyf = nh.param("Cyf", 444);
  this->Cyr = nh.param("Cyr", 444);
  this->m = nh.param("m", 320);
  this->lr = this->COG * this->l_wheelbase;
  this->lf = (1 - this->COG) * this->l_wheelbase;
}

// The READ function
void Gen4HWInterface::read(ros::Duration& elapsed_time)
{
  joint_velocity_[drive_joint0_id] = this->cur_velocity_axis0;
  joint_velocity_[drive_joint1_id] = this->cur_velocity_axis1;
  joint_position_[steering_joint_id] = this->cur_steering;
}

// The WRITE function
void Gen4HWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  if (this->is_running == true)
  {
    publish_steering_msg(joint_position_command_[steering_joint_id]);
    publish_torque_msg(joint_effort_command_[drive_joint0_id]);
  }
  else
  {
    publish_torque_msg(0.0);
    publish_torque_msg(0.0);
  }
}

void Gen4HWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  vel_jnt_sat_interface_.enforceLimits(period);
  eff_jnt_sat_interface_.enforceLimits(period);
}

void Gen4HWInterface::state_change(const ugr_msgs::State::ConstPtr& msg)
{
  if (msg->scope == "autonomous")
  {
    this->is_running = msg->cur_state == "asdrive";
  }
  else if (msg->scope == "slam" && msg->cur_state == "finished")
  {
    this->is_running = false;
  }
}

// Callback for the CAN messages: axis0, axis1 and steering
void Gen4HWInterface::can_callback_axis0(const std_msgs::Float32::ConstPtr& msg)
{
  handle_vel_msg(msg, 1);
}

void Gen4HWInterface::can_callback_axis1(const std_msgs::Float32::ConstPtr& msg)
{
  handle_vel_msg(msg, 2);
}

void Gen4HWInterface::can_callback_steering(const std_msgs::Float32::ConstPtr& msg)
{
  handle_steering_msg(msg);
}

void Gen4HWInterface::handle_vel_msg(const std_msgs::Float32::ConstPtr& msg, uint32_t axis_id)
{
  double motor_speed = msg->data;  // rpm

  // Set cur_velocity_axis
  if (axis_id == 1)
  {
    this->cur_velocity_axis0 = 2 * M_PI * motor_speed / 60;  // rad/s
  }
  else if (axis_id == 2)
  {
    this->cur_velocity_axis1 = 2 * M_PI * motor_speed / 60;  // rad/s
  }
}

void Gen4HWInterface::handle_steering_msg(const std_msgs::Float32::ConstPtr& msg)
{
  this->cur_steering = msg->data;
}

void Gen4HWInterface::publish_steering_msg(float steering)
{
  // Convert [-3.14, 3.14] to a steering range [-steer_max_step, steer_max_step]

  steering = steering / 3.14 * steer_max_step;
  // TODO place steering commmand on ros topic
}

void Gen4HWInterface::publish_torque_msg(float axis)
{
  float mean_axis_speed = (this->cur_velocity_axis0 + this->cur_velocity_axis1) / 2;
  float car_speed =
      mean_axis_speed / this->gear_ratio * M_PI * this->wheel_diameter / 60;  // m/s, mean speed if no slip

  // no TV at low speeds
  if (car_speed > 5 && this->use_torque_vectoring == true)
  {
    // float dT = this->torque_vectoring();
    // float axis0 = axis - dT / 2;
    // float axis1 = axis + dT / 2;
  }

  // TODO : look for the can id's in dbc file (id's for cmd_target_speed or cmd_target_position or (cmd_target_current))
  // TODO : send the motor commands in can_msgs::Frame format on the can bus

  // make ros can frame and send on can
  // can_msgs::Frame can_msg;
  // can_msg.id = can_id;
  // can_msg.data = axis0;
  // can_msg.dlc = 8;

  // can_pub.publish(can_msg);
}

void Gen4HWInterface::yaw_rate_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  this->yaw_rate = msg->angular_velocity.z;
}

float Gen4HWInterface::torque_vectoring()
{
  // returns the torque distribution for the left and right wheel
  float mean_axis_speed = (this->cur_velocity_axis0 + this->cur_velocity_axis1) / 2;
  float car_speed = mean_axis_speed / this->gear_ratio * M_PI * this->wheel_diameter / 60;  // m/s
  float yaw_rate_desired = 0.0;

  // calculate the understeer gradient
  float Ku = this->lr * this->m / (this->Cyf * (this->lf + this->lr)) -
             this->lf * this->m / (this->Cyr * (this->lf + this->lr));

  // calculate the desired yaw rate, add safety for division by zero
  if (abs(this->lr + this->lf + Ku * pow(car_speed, 2)) > 0.0001)
    ;
  {
    yaw_rate_desired = car_speed / (this->lr + this->lf + Ku * pow(car_speed, 2)) * this->cur_steering;
  }

  float yaw_rate_error = yaw_rate_desired - this->yaw_rate;

  // PI(D) controller calculates the difference in torque dT, based on the yaw rate error
  float now_time = ros::Time::now().toSec();
  this->integral += yaw_rate_error * (now_time - this->prev_time);
  float difference = (yaw_rate_error - this->prev_error) / (now_time - this->prev_time);

  float dT =
      std::min(std::max(this->Kp * yaw_rate_error + this->Ki * this->integral + this->Kd * difference, -this->max_dT),
               this->max_dT);

  this->prev_error = yaw_rate_error;
  this->prev_time = now_time;

  return dT;
}

}  // namespace gen4_control
