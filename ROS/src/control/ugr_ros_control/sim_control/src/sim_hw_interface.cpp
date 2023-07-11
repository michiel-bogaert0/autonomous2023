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

#include <sim_control/sim_hw_interface.h>
#include <tuple>

namespace sim_control
{
SimHWInterface::SimHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ugr_ros_control::GenericHWInterface(nh, urdf_model)
{
  this->model = new BicycleModel();

  ROS_INFO_NAMED("sim_hw_interface", "SimHWInterface Ready.");
}

void SimHWInterface::init()
{
  // First do parent init
  ugr_ros_control::GenericHWInterface::init();

  // Now check if configured joints are actually there. Also remembe joint id
  std::string drive_joint_name = nh_.param<std::string>("hardware_interface/joints_config/drive_joint", "axis0");
  std::string steering_joint_name = nh_.param<std::string>("hardware_interface/joints_config/steering_joint", "axis_steering");

  drive_joint_id = std::find(joint_names_.begin(), joint_names_.end(), drive_joint_name) - joint_names_.begin();
  steering_joint_id = std::find(joint_names_.begin(), joint_names_.end(), steering_joint_name) - joint_names_.begin();

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
}

void SimHWInterface::read(ros::Duration& elapsed_time)
{
  // Gets done in write because of simulation
  // So empty function!
}

void SimHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // Feed to bicycle model
  auto modelState = this->model->update(elapsed_time.toSec(), joint_effort_command_[drive_joint_id],
                                       joint_effort_command_[steering_joint_id]);

  // Write result back
  joint_velocity_[drive_joint_id] = this->model->get_forward_velocity();
  joint_position_[steering_joint_id] = this->model->get_steering_angle();
}

void SimHWInterface::enforceLimits(ros::Duration& period)
{
  // TODO
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace sim_control
