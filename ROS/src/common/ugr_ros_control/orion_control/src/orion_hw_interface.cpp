#include <orion_control/orion_hw_interface.hpp>
#include <can_msgs/Frame.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <random>
#include <tuple>
#include <math.h>

namespace orion_control
{
OrionHWInterface::OrionHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ugr_ros_control::GenericHWInterface(nh, urdf_model)
{
  ros::NodeHandle n("~");

  ROS_INFO_NAMED("orion_hw_interface", "OrionHWInterface Ready.");
}

void OrionHWInterface::init()
{
  // First do parent init
  ugr_ros_control::GenericHWInterface::init();

  ros::NodeHandle nh("~");

  // Publishers
  this->can_axis0_pub = nh.advertise<can_msgs::Frame>("/output/axis0", 10);
  this->can_axis1_pub = nh.advertise<can_msgs::Frame>("/output/axis1", 10);

  this->can_servo_pub = nh.advertise<std_msgs::Float32>("/output/servo", 10);

  // Velocity estimate by rear wheels
  this->vel_pub0 = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/output/vel0", 10);
  this->vel_pub1 = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/output/vel1", 10);

  // Subscribers
  // TODO update

  // axis0 is rechts
  this->can_axis0_sub = nh.subscribe<std_msgs::Int64>("/ugr/can/mc_right/processed/actual_erpm", 1,
                                                      &OrionHWInterface::can_callback_axis0, this);
  this->can_axis1_sub = nh.subscribe<std_msgs::Int64>("/ugr/can/mc_left/processed/actual_erpm", 1,
                                                      &OrionHWInterface::can_callback_axis1, this);

  // Servo position sub
  this->can_steering_sub =
      nh.subscribe<std_msgs::Float32>("/input/servo", 1, &OrionHWInterface::can_callback_steering, this);

  this->jaw_rate_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, &OrionHWInterface::yaw_rate_callback, this);

  // Safety
  this->state_sub = nh.subscribe<ugr_msgs::State>("/state/car", 1, &OrionHWInterface::state_change, this);

  // Now check if configured joints are actually there. Also remember joint id
  std::string axis0_joint_id_name = nh_.param<std::string>("hardware_interface/axis0_joint", "axis0");
  std::string axis1_joint_id_name = nh_.param<std::string>("hardware_interface/axis1_joint", "axis1");
  std::string steering_joint_name = nh_.param<std::string>("hardware_interface/steering_joint", "axis_steering");

  this->axis0_frame = nh.param("axis0/frame", std::string("ugr/car_base_link/axis0"));
  this->axis1_frame = nh.param("axis1/frame", std::string("ugr/car_base_link/axis1"));
  this->wheel_diameter = nh.param("wheel_diameter", 16.0 * 2.54 / 100.0);  // in m
  this->gear_ratio = nh.param("gear_ratio", 3.405);
  this->n_polepairs = nh.param("n_polepairs", 10);

  axis0_joint_id = std::find(joint_names_.begin(), joint_names_.end(), axis0_joint_id_name) - joint_names_.begin();
  axis1_joint_id = std::find(joint_names_.begin(), joint_names_.end(), axis1_joint_id_name) - joint_names_.begin();
  steering_joint_id = std::find(joint_names_.begin(), joint_names_.end(), steering_joint_name) - joint_names_.begin();

  ROS_INFO_STREAM("Drive joint id: " << axis0_joint_id << " (" << axis0_joint_id_name << "), " << axis1_joint_id << " ("
                                     << axis1_joint_id_name << "), "
                                     << "< Steering joint id: " << steering_joint_id << " (" << steering_joint_name
                                     << ")");

  if (axis0_joint_id >= joint_names_.size())
  {
    ROS_ERROR("Error: the parameter 'hardware_interface/joints_config/axis0_joint_id' must be given");
    throw std::invalid_argument("hardware_interface/joints_config/axis0_joint_id must be given");
  }

  if (axis1_joint_id >= joint_names_.size())
  {
    ROS_ERROR("Error: the parameter 'hardware_interface/joints_config/axis1_joint_id' must be given");
    throw std::invalid_argument("hardware_interface/joints_config/axis1_joint_id must be given");
  }

  if (steering_joint_id >= joint_names_.size())
  {
    ROS_ERROR("Error: the parameter 'hardware_interface/joints_config/steering_joint' must be given");
    throw std::invalid_argument("hardware_interface/joints_config/steering_joint must be given");
  }

  ROS_INFO_NAMED("orion_hw_interface", "OrionHWInterface init'd.");

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

bool OrionHWInterface::canSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                 const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  return true;
}

// The READ function
void OrionHWInterface::read(ros::Duration& elapsed_time)
{
  joint_velocity_[axis0_joint_id] = (this->cur_velocity_axis0);
  joint_velocity_[axis1_joint_id] = (this->cur_velocity_axis1);
  joint_position_[steering_joint_id] = this->cur_steering;
}

// The WRITE function
void OrionHWInterface::write(ros::Duration& elapsed_time)
{
  ROS_INFO_STREAM(this->printCommandHelper());
  ROS_INFO_STREAM(this->printStateHelper());
  // Safety
  enforceLimits(elapsed_time);

  if (this->is_running == true)
  {
    send_torque_on_can(joint_effort_command_[axis0_joint_id], 0);
    send_torque_on_can(joint_effort_command_[axis1_joint_id], 1);

    can_msgs::Frame msg;
    msg.header.stamp = ros::Time::now();
    msg.id = 0x24FF;
    msg.is_extended = true;
    msg.dlc = 8;
    msg.data = { 1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    this->can_axis0_pub.publish(msg);
    this->can_axis1_pub.publish(msg);
    publish_steering_msg(-1 * joint_position_command_[steering_joint_id]);
  }
  else
  {
    send_torque_on_can(0, 0);
    send_torque_on_can(0, 1);
  }
}

void OrionHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  vel_jnt_sat_interface_.enforceLimits(period);
  eff_jnt_sat_interface_.enforceLimits(period);
}

void OrionHWInterface::state_change(const ugr_msgs::State::ConstPtr& msg)
{
  this->is_running = msg->cur_state == "r2d";
}

// Callback for the CAN messages: axis0, axis1 and steering
void OrionHWInterface::can_callback_axis0(const std_msgs::Int64::ConstPtr& msg)
{
  float motor_rpm = msg->data / this->n_polepairs;
  this->cur_velocity_axis0 = 2 * M_PI * motor_rpm / 60 / this->gear_ratio * (this->wheel_diameter / 2);  // rpm to rad/s
  this->handle_vel_msg();

  geometry_msgs::TwistWithCovarianceStamped twist_msg = geometry_msgs::TwistWithCovarianceStamped();
  twist_msg.header.frame_id = this->axis0_frame;
  twist_msg.header.stamp = ros::Time::now();
  twist_msg.twist.covariance = { 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  twist_msg.twist.twist = geometry_msgs::Twist();

  twist_msg.twist.twist.linear.x = this->cur_velocity_axis0;  // rad/s to m/s

  vel_pub0.publish(twist_msg);
}

void OrionHWInterface::can_callback_axis1(const std_msgs::Int64::ConstPtr& msg)
{
  float motor_rpm = msg->data / this->n_polepairs;
  this->cur_velocity_axis1 =
      ((2.0 * M_PI * motor_rpm / 60.0) / this->gear_ratio) * (this->wheel_diameter / 2.0);  // rpm to rad/s
  this->handle_vel_msg();

  geometry_msgs::TwistWithCovarianceStamped twist_msg = geometry_msgs::TwistWithCovarianceStamped();
  twist_msg.header.frame_id = this->axis1_frame;
  twist_msg.header.stamp = ros::Time::now();
  twist_msg.twist.covariance = { 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  twist_msg.twist.twist = geometry_msgs::Twist();

  twist_msg.twist.twist.linear.x = this->cur_velocity_axis1;

  vel_pub1.publish(twist_msg);
}

void OrionHWInterface::can_callback_steering(const std_msgs::Float32::ConstPtr& msg)
{
  // TODO: correct conversion !!!!!!
  this->cur_steering = msg->data;

  // Not sure if handle_steering_msg is necessary (not used by any other node atm)
}

void OrionHWInterface::handle_vel_msg()
{
  // Publish the car velocity (based on mean motor velocity) as a twist msg (used by SLAM)
}

void OrionHWInterface::publish_steering_msg(float steering)
{
  // Convert [-3.14, 3.14] to a steering range [-steer_max_step, steer_max_step]

  // TODO: correct conversion !!!!!!

  std_msgs::Float32 msg;
  msg.data = steering;

  this->can_servo_pub.publish(msg);
}

void OrionHWInterface::publish_torque_msg(float axis)
{
  // float cur_vel_rear = joint_velocity_[drive_joint_id];
  // float car_vel_estimate =
  //     cur_vel_rear / this->gear_ratio * M_PI * this->wheel_diameter / 60;  // m/s, mean vel if no slip

  float axis0 = axis;
  float axis1 = axis;

  // // no TV at low vel
  // if (car_vel_estimate > 5 && this->use_torque_vectoring == true)
  // {
  //   float dT = this->torque_vectoring();
  //   axis0 = axis - dT / 2;
  //   axis1 = axis + dT / 2;
  // }

  // send on CAN
  send_torque_on_can(axis0, 0);
  send_torque_on_can(axis1, 1);
}

void OrionHWInterface::send_torque_on_can(float axis, int id)
{
  int16_t axis_int = axis * 10;

  if (abs(axis) < 1.0)
  {
    axis_int = 0.0;
  }

  // create publish message
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.id = 0x1A45 + id;
  msg.is_extended = true;
  msg.dlc = 2;
  msg.data = { (axis_int >> 8) & 0xFF, axis_int & 0xFF, 0, 0, 0, 0, 0, 0 };

  // publish
  if (id == 0)
  {
    this->can_axis0_pub.publish(msg);
  }
  else
  {
    this->can_axis1_pub.publish(msg);
  }
}

void OrionHWInterface::yaw_rate_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  this->yaw_rate = msg->angular_velocity.z;
}

float OrionHWInterface::torque_vectoring()
{
  return 0;
  // returns the torque distribution for the left and right wheel
  // float cur_velocity_rear = joint_velocity_[drive_joint_id];
  // float car_vel = cur_velocity_rear / this->gear_ratio * M_PI * this->wheel_diameter / 60;  // m/s
  // float yaw_rate_desired = 0.0;

  // // calculate the understeer gradient
  // float Ku = this->lr * this->m / (this->Cyf * (this->lf + this->lr)) -
  //            this->lf * this->m / (this->Cyr * (this->lf + this->lr));

  // // calculate the desired yaw rate, add safety for division by zero
  // if (abs(this->lr + this->lf + Ku * pow(car_vel, 2)) > 0.0001)
  //   ;
  // {
  //   yaw_rate_desired = car_vel / (this->lr + this->lf + Ku * pow(car_vel, 2)) * this->cur_steering;
  // }

  // float yaw_rate_error = yaw_rate_desired - this->yaw_rate;

  // // PI(D) controller calculates the difference in torque dT, based on the yaw rate error
  // double now_time = ros::Time::now().toSec();
  // this->integral += yaw_rate_error * (now_time - this->prev_time);
  // float difference = (yaw_rate_error - this->prev_error) / (now_time - this->prev_time);

  // float dT =
  //     std::min(std::max(this->Kp * yaw_rate_error + this->Ki * this->integral + this->Kd * difference, -this->max_dT),
  //              this->max_dT);

  // this->prev_error = yaw_rate_error;
  // this->prev_time = now_time;

  // return dT;
}

}  // namespace orion_control
