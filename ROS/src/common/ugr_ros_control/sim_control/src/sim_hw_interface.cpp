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

#include <sim_control/sim_hw_interface.hpp>
#include <random>
#include <tuple>

namespace sim_control
{
SimHWInterface::SimHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ugr_ros_control::GenericHWInterface(nh, urdf_model)
{
  ros::NodeHandle n("~");

  this->model = new BicycleModel(n.param<double>("R", 0.2), n.param<double>("L", 1.6), n.param<double>("Lr", 0.7),
                                 n.param<double>("mu", 0.1), n.param<double>("DC", 0.025));

  ROS_INFO_NAMED("sim_hw_interface", "SimHWInterface Ready.");
}

void SimHWInterface::init()
{
  // First do parent init
  ugr_ros_control::GenericHWInterface::init();

  ros::NodeHandle nh("~");

  nh.param("world_frame", world_frame, std::string("ugr/map"));
  nh.param("base_link_frame", base_link_frame, std::string("ugr/car_base_link"));
  nh.param("gt_base_link_frame", gt_base_link_frame, std::string("ugr/gt_base_link"));

  double gt_publish_rate, encoder_publish_rate, imu_publish_rate;
  nh.param("publish_rates/gt", gt_publish_rate, 200.0);
  nh.param("publish_rates/encoder", encoder_publish_rate, 30.0);
  nh.param("publish_rates/imu", imu_publish_rate, 90.0);

  std::vector<double> encoder_noise_default = { 0, 0.05, 0.05 };
  std::vector<double> imu_acceleration_noise_default = { 0.0, 0.1, 0.01 };
  std::vector<double> imu_angular_velocity_noise_default = { 0, 0.1, 0.01 };

  nh.param("noise/encoder", encoder_noise, encoder_noise_default);
  nh.param("noise/imu_acceleration", imu_acceleration_noise, imu_acceleration_noise_default);
  nh.param("noise/imu_angular_velocity", imu_angular_velocity_noise, imu_angular_velocity_noise_default);

  this->gt_pub = nh.advertise<nav_msgs::Odometry>("/output/gt_odometry", 5);
  this->encoder_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/output/encoder0", 5);
  this->imu_pub = nh.advertise<sensor_msgs::Imu>("/output/imu0", 5);
  this->TV_axis0 = nh.advertise<std_msgs::Float32>("/ugr/car/TV_axis0", 0.0);
  this->TV_axis1 = nh.advertise<std_msgs::Float32>("/ugr/car/TV_axis1", 0.0);

  this->gt_timer = nh.createTimer(ros::Duration(1 / gt_publish_rate), &SimHWInterface::publish_gt, this);
  this->encoder_timer = nh.createTimer(ros::Duration(1 / encoder_publish_rate), &SimHWInterface::publish_encoder, this);
  this->imu_timer = nh.createTimer(ros::Duration(1 / imu_publish_rate), &SimHWInterface::publish_imu, this);

  // Now check if configured joints are actually there. Also remembe joint id
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

  ROS_INFO_NAMED("sim_hw_interface", "SimHWInterface init'd.");

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
  this->l_wheelbase = nh.param("l_wheelbase", 2);
  this->COG = nh.param("COG", 0.5);
  this->Cyf = nh.param("Cyf", 444);
  this->Cyr = nh.param("Cyr", 444);
  this->m = nh.param("m", 320);
  this->lr = this->COG * this->l_wheelbase;
  this->lf = (1 - this->COG) * this->l_wheelbase;
}

void SimHWInterface::read(ros::Duration& elapsed_time)
{
  auto car_state = this->model->get_car_state();
  ROS_DEBUG_STREAM("x: " << std::get<0>(car_state) << " y: " << std::get<1>(car_state)
                         << " theta: " << std::get<2>(car_state) << " v: " << this->model->get_forward_velocity());

  // Effort gets applied immediately (assumption)
  joint_effort_[drive_joint_id] = joint_effort_command_[drive_joint_id];
  joint_velocity_[steering_joint_id] = joint_velocity_command_[steering_joint_id];
  joint_velocity_[drive_joint_id] = this->model->get_wheel_angular_velocity();
  joint_position_[steering_joint_id] = this->model->get_steering_angle();
}

void SimHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // TODO add logic for when using a feedforward velocity controller instead of effort interface / joint velocity controller

  // Torque control
  float axis0 = joint_effort_command_[drive_joint_id];
  float car_speed = axis0 * M_PI * 0.2 / 60;  // m/s (wheeldiam = 0.2m, no gear ratio)
  float delta = joint_position_[steering_joint_id];

  std_msgs::Float32 msg0;
  msg0.data = axis0;
  std_msgs::Float32 msg1;
  msg1.data = axis0;

  // no TV at low speeds
  if (car_speed > 5.0)
  {
    float dT = torque_vectoring(axis0, delta);
    msg0.data -= dT / 2;
    msg1.data += dT / 2;
  }

  this->TV_axis0.publish(msg0);
  this->TV_axis1.publish(msg1);

  // Feed to bicycle model
  this->model->update(elapsed_time.toSec(), joint_effort_command_[drive_joint_id],
                      joint_velocity_command_[steering_joint_id]);
}

void SimHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  vel_jnt_sat_interface_.enforceLimits(period);
  eff_jnt_sat_interface_.enforceLimits(period);
}

void SimHWInterface::publish_gt(const ros::TimerEvent&)
{
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = this->world_frame;
  odom.child_frame_id = this->gt_base_link_frame;

  auto state = this->model->get_car_state();

  odom.pose.pose.position.x = std::get<0>(state);
  odom.pose.pose.position.y = std::get<1>(state);

  tf2::Quaternion q;
  q.setRPY(0, 0, std::get<2>(state));
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = this->model->get_forward_velocity();
  odom.twist.twist.angular.z = this->model->get_angular_velocity();

  this->gt_pub.publish(odom);

  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time::now();
  t.header.frame_id = this->world_frame;
  t.child_frame_id = this->gt_base_link_frame;
  t.transform.translation.x = std::get<0>(state);
  t.transform.translation.y = std::get<1>(state);
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(t);
}

void SimHWInterface::apply_noise_and_quantise(float& x, std::vector<double> noise)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> d(noise[0], noise[1]);
  x += d(gen);
  x = round(x / noise[2]) * noise[2];
}

void SimHWInterface::publish_encoder(const ros::TimerEvent&)
{
  geometry_msgs::TwistWithCovarianceStamped twist;
  twist.header.stamp = ros::Time::now();
  twist.header.frame_id = this->base_link_frame;
  float noisy_v = this->model->get_forward_velocity();
  apply_noise_and_quantise(noisy_v, this->encoder_noise);
  twist.twist.twist.linear.x = noisy_v;

  this->encoder_pub.publish(twist);
}

void SimHWInterface::publish_imu(const ros::TimerEvent&)
{
  sensor_msgs::Imu imu;
  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = this->base_link_frame;
  float noisy_omega = this->model->get_angular_velocity();
  float noisy_a = this->model->get_acceleration();
  apply_noise_and_quantise(noisy_omega, this->imu_angular_velocity_noise);
  apply_noise_and_quantise(noisy_a, this->imu_acceleration_noise);
  imu.linear_acceleration.x = noisy_a;
  imu.angular_velocity.z = noisy_omega;

  this->imu_pub.publish(imu);
  this->yaw_rate = noisy_omega;
}

float SimHWInterface::torque_vectoring(float axis0, float delta)
{
  // returns the torque distribution for the left and right wheel

  float car_speed = axis0 * M_PI * 0.2 / 60;  // m/s (wheeldiam = 0.2m, no gear ratio)
  float yaw_rate_desired = 0.0;

  // calculate the understeer gradient
  float Ku = this->lr * this->m / (this->Cyf * (this->lf + this->lr)) -
             this->lf * this->m / (this->Cyr * (this->lf + this->lr));
  float dT = 0.0;

  // calculate the desired yaw rate, add safety for division by zero
  if (abs(this->lr + this->lf + Ku * pow(car_speed, 2)) > 0.0001)
    ;
  {
    yaw_rate_desired = car_speed / (this->lr + this->lf + Ku * pow(car_speed, 2)) * delta;
  }

  float yaw_rate_error = yaw_rate_desired - this->yaw_rate;

  // // PID controller calculates the difference in torque dT, based on the yaw rate error
  // // Recommended to use D = 0
  float now_time = ros::Time::now().toSec();
  this->integral += yaw_rate_error * (now_time - this->prev_time);
  // float difference = (yaw_rate_error - this->prev_error) / (now_time - this->prev_time);
  // difference = 0.0, otherwise the difference is inf
  float difference = 0.0;

  dT = std::min(std::max(this->Kp * yaw_rate_error + this->Ki * this->integral + this->Kd * difference, -this->max_dT),
                this->max_dT);

  this->prev_error = yaw_rate_error;
  this->prev_time = now_time;

  return dT;
}

}  // namespace sim_control
