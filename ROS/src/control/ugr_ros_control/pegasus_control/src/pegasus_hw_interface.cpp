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

#include <pegasus_control/pegasus_hw_interface.h>
#include <random>
#include <tuple>

const int CAN_NODE_ID = 0xE0;
const int CAN_STEER_ID = 0x3;

namespace pegasus_control
{
PegasusHWInterface::PegasusHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ugr_ros_control::GenericHWInterface(nh, urdf_model)
{
  ros::NodeHandle n("~");

  // this->model = new BicycleModel(n.param<double>("R", 0.1), n.param<double>("L", 1.0), n.param<double>("Lr", 0.5),
  //                                n.param<double>("mu", 0.1), n.param<double>("DC", 0.8));

  ROS_INFO_NAMED("pegasus_hw_interface", "PegasusHWInterface Ready.");
}

void PegasusHWInterface::init()
{
  // First do parent init
  ugr_ros_control::GenericHWInterface::init();

  ros::NodeHandle nh("~");

  nh.param("world_frame", world_frame, std::string("ugr/map"));
  nh.param("base_link_frame", base_link_frame, std::string("ugr/car_base_link"));

  float steer_max_step;
  nh.param("steer_max_step", steer_max_step, 1600);

  ros::Publisher bus = nh.advertise<can_msgs::msg::Frame>("/output/can", 10);

  // Now check if configured joints are actually there. Also remember joint id
  std::string drive_joint_name = nh_.param<std::string>("hardware_interface/drive_joint", "axis0");
  std::string steering_joint_name = nh_.param<std::string>("hardware_interface/steering_joint", "axis_steering");

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
  // get input


}

void PegasusHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // Open loop steering so just couple back
  joint_position_[steering_joint_id] = joint_position_command_[steering_joint_id];
  publish_steering_msg(joint_position_command_[steering_joint_id]);

  publish_vel_msg(joint_velocity_command[drive_joint_id], 1);
  publish_vel_msg(joint_velocity_command[drive_joint_id], 2);


}

void PegasusHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
}

void PegasusHWInterface::publish_steering_msg(float steering)
{
  // Convert [-1, 1] to a steering range [-steer_max_step, steer_max_step]
  steering = steering * steer_max_step;
  int id = CAN_NODE_ID << 2 | CAN_STEER_ID;

  cantools::odrive_set_input_steering_t* msg;

  cantools::odrive_set_input_steering_init(msg);

  &msg->input_steering = steering;  

  uint8_t encoded_data[2];
  cantools::odrive_set_input_vel_pack(encoded_data, msg, sizeof(encoded_data));
  can_msgs::msg::Frame msg;
  msg.id = id;
  msg.data = encoded_data;
  msg.dlc = encoded_data.size();

  bus->publish(msg);
}

void PegasusHWInterface::publish_vel_msg(float vel, int axis)
{
  // axis = 1 is right, axis = 2 is left
  cantools::odrive_set_input_vel_t* msg;

  cantools::odrive_set_input_vel_init(msg);

  &msg->input_vel = vel;
  &msg->input_torque_ff = 0;

  uint8_t encoded_data[8];
  cantools::odrive_set_input_vel_pack(encoded_data, msg, sizeof(encoded_data));

  int can_id = axis << 5 | 0x00D;

  can_msgs::msg::Frame msg;
  msg.id = can_id;
  msg.data = encoded_data;
  msg.dlc = encoded_data.size();

  bus.publish(msg);
}

}  // namespace pegasus_control
