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

#include <pegasus_control/pegasus_hw_interface.hpp>
#include <random>
#include <tuple>
#include <math.h>

const uint32_t CAN_NODE_ID = 0xE0;
const uint32_t CAN_STEER_ID = 0x3;

const uint32_t ODRIVE_VELOCITY_CONTROL_MODE = 2;
const uint32_t ODRIVE_PASSTHROUGH_INPUT_MODE = 1;

namespace pegasus_control
{
PegasusHWInterface::PegasusHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ugr_ros_control::GenericHWInterface(nh, urdf_model)
{
  ros::NodeHandle n("~");

  ROS_INFO_NAMED("pegasus_hw_interface", "PegasusHWInterface Ready.");
}

void PegasusHWInterface::init()
{
  // First do parent init
  ugr_ros_control::GenericHWInterface::init();

  ros::NodeHandle nh("~");

  nh.param("steer_max_step", steer_max_step, 1600.0f);

  this->can_pub = nh.advertise<can_msgs::Frame>("/output/can", 10);
  this->can_sub = nh.subscribe<can_msgs::Frame>("/input/can", 1, &PegasusHWInterface::can_callback, this);

  this->state_sub = nh.subscribe<ugr_msgs::State>("/state", 1, &PegasusHWInterface::state_change, this);

  this->vel_right_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/output/vel0", 10);
  this->vel_left_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/output/vel1", 10);

  // Now check if configured joints are actually there. Also remember joint id
  std::string drive_joint_name = nh_.param<std::string>("hardware_interface/drive_joint", "axis0");
  std::string steering_joint_name = nh_.param<std::string>("hardware_interface/steering_joint", "axis_steering");

  this->axis0_frame = nh.param("axis0/frame", std::string("ugr/car_base_link/axis0"));
  this->axis1_frame = nh.param("axis1/frame", std::string("ugr/car_base_link/axis1"));
  this->wheel_diameter = nh.param("wheel_diameter", 8.0 * 2.54 / 100.0);  // in m

  drive_joint_id = std::find(joint_names_.begin(), joint_names_.end(), drive_joint_name) - joint_names_.begin();
  steering_joint_id = std::find(joint_names_.begin(), joint_names_.end(), steering_joint_name) - joint_names_.begin();

  ROS_INFO_STREAM("Drive joint id: " << drive_joint_id << " Steering joint id: " << steering_joint_id);

  if (drive_joint_id >= joint_names_.size())
  {
    ROS_ERROR("Error: the parameter 'hardware_interface/joints_config/drive_joint' must be given");
    throw std::invalid_argument("hardware_interface/joints_config/drive_joint must be given");
  }

  if (steering_joint_id >= joint_names_.size())
  {
    ROS_ERROR("Error: the parameter 'hardware_interface/joints_config/steering_joint' must be given");
    throw std::invalid_argument("hardware_interface/joints_config/steering_joint must be given");
  }

  ROS_INFO_NAMED("pegasus_hw_interface", "PegasusHWInterface init'd.");
}

void PegasusHWInterface::read(ros::Duration& elapsed_time)
{
  joint_velocity_[drive_joint_id] = this->cur_velocity;
}

void PegasusHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // Open loop steering so just couple back
  joint_position_[steering_joint_id] = joint_position_command_[steering_joint_id];

  if (this->is_running == true)
  {
    publish_steering_msg(joint_position_command_[steering_joint_id]);

    publish_vel_msg(joint_velocity_command_[drive_joint_id], 1);
    publish_vel_msg(-joint_velocity_command_[drive_joint_id], 2);
  }
  else
  {
    publish_vel_msg(0, 1);
    publish_vel_msg(0, 2);
  }
}

bool PegasusHWInterface::canSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                   const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  // TODO
  return true;
}

void PegasusHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                  const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  cantools::odrive_set_controller_mode_t mode_msg;

  cantools::odrive_set_controller_mode_init(&mode_msg);

  mode_msg.input_mode = ODRIVE_PASSTHROUGH_INPUT_MODE;
  mode_msg.control_mode = ODRIVE_VELOCITY_CONTROL_MODE;

  uint8_t encoded_data[8];
  cantools::odrive_set_controller_mode_pack(encoded_data, &mode_msg, sizeof(encoded_data));

  boost::array<unsigned char, 8> converted_data;  // Different type needed for can msg
  for (size_t i = 0; i < converted_data.size(); ++i)
  {
    converted_data[i] = encoded_data[i];
  }

  can_msgs::Frame can_msg;
  can_msg.data = converted_data;
  can_msg.dlc = 8;

  uint32_t can_id = 1 << 5 | 0x00D;  // Right
  can_msg.id = can_id;

  can_pub.publish(can_msg);

  can_id = 2 << 5 | 0x00D;  // Left
  can_msg.id = can_id;

  can_pub.publish(can_msg);
}

void PegasusHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  vel_jnt_sat_interface_.enforceLimits(period);
  eff_jnt_sat_interface_.enforceLimits(period);
}

void PegasusHWInterface::state_change(const ugr_msgs::State::ConstPtr& msg)
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

void PegasusHWInterface::can_callback(const can_msgs::Frame::ConstPtr& msg)
{
  uint32_t axis_id = msg->id >> 5;
  if (axis_id == 1 || axis_id == 2)
  {
    uint32_t cmd_id = msg->id & 0b11111;

    if (cmd_id == 9)
    {
      // TODO: publish diagnostics
      handle_vel_msg(msg, axis_id);
    }
  }
}

void PegasusHWInterface::handle_vel_msg(const can_msgs::Frame::ConstPtr& msg, uint32_t axis_id)
{
  cantools::odrive_get_encoder_estimates_t vel_msg;

  cantools::odrive_get_encoder_estimates_init(&vel_msg);

  uint8_t encoded_data[8];

  std::copy(msg->data.begin(), msg->data.end(), encoded_data);
  cantools::odrive_get_encoder_estimates_unpack(&vel_msg, encoded_data, msg->dlc);

  geometry_msgs::TwistWithCovarianceStamped twist_msg = geometry_msgs::TwistWithCovarianceStamped();
  twist_msg.header.frame_id = (axis_id == 1) ? this->axis0_frame : this->axis1_frame;
  twist_msg.header.stamp = ros::Time::now();
  twist_msg.twist = geometry_msgs::TwistWithCovariance();

  // TODO Use actual covariance measurements (first we need data to estimate these)
  twist_msg.twist.covariance = { 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  twist_msg.twist.twist = geometry_msgs::Twist();

  double speed = vel_msg.vel_estimate;  // in rev/s
  speed *= M_PI * this->wheel_diameter;

  twist_msg.twist.twist.linear.x = (axis_id == 1) ? speed : -speed;  // The left wheel is inverted

  if (axis_id == 1)
  {
    this->cur_velocity = vel_msg.vel_estimate * 2 * M_PI;  // rad/s
    vel_right_pub.publish(twist_msg);
  }
  else if (axis_id == 2)
  {
    vel_left_pub.publish(twist_msg);
  }
}

void PegasusHWInterface::publish_steering_msg(float steering)
{
  // Convert [-3.14, 3.14] to a steering range [-steer_max_step, steer_max_step]

  steering = steering / -3.14 * steer_max_step;
  uint32_t id = CAN_NODE_ID << 2 | CAN_STEER_ID;

  cantools::odrive_set_input_steering_t msg;

  cantools::odrive_set_input_steering_init(&msg);

  msg.input_steering = (int)steering;

  uint8_t encoded_data[8];
  cantools::odrive_set_input_steering_pack(encoded_data, &msg, sizeof(encoded_data));

  boost::array<unsigned char, 8> converted_data;  // Different type needed for can msg
  for (size_t i = 0; i < converted_data.size(); i++)
  {
    converted_data[i] = encoded_data[i];
  }

  can_msgs::Frame can_msg;
  can_msg.id = id;
  can_msg.data = converted_data;
  can_msg.dlc = 2;

  can_pub.publish(can_msg);
}

void PegasusHWInterface::publish_vel_msg(float vel, int axis)
{
  // axis = 1 is right, axis = 2 is left
  cantools::odrive_set_input_vel_t msg;

  cantools::odrive_set_input_vel_init(&msg);

  msg.input_vel = vel / (2 * M_PI);
  msg.input_torque_ff = 0;

  uint8_t encoded_data[8];
  cantools::odrive_set_input_vel_pack(encoded_data, &msg, sizeof(encoded_data));

  uint32_t can_id = axis << 5 | 0x00D;

  boost::array<unsigned char, 8> converted_data;  // Different type needed for can msg
  for (size_t i = 0; i < converted_data.size(); i++)
  {
    converted_data[i] = encoded_data[i];
  }

  can_msgs::Frame can_msg;
  can_msg.id = can_id;
  can_msg.data = converted_data;
  can_msg.dlc = 8;

  can_pub.publish(can_msg);
}

}  // namespace pegasus_control
