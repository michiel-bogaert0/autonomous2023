/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
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
 *   * Neither the name of the PickNik LLC nor the names of its
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

/* Author: Dave Coleman <dave@dav.ee>, Andy McEvoy
   Desc:   Tweak a TF transform using a keyboard
*/

#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#define KEYCODE_a 0x61
#define KEYCODE_b 0x62
#define KEYCODE_c 0x63
#define KEYCODE_d 0x64
#define KEYCODE_e 0x65
#define KEYCODE_f 0x66
#define KEYCODE_g 0x67
#define KEYCODE_h 0x68
#define KEYCODE_i 0x69
#define KEYCODE_j 0x6a
#define KEYCODE_k 0x6b
#define KEYCODE_l 0x6c
#define KEYCODE_m 0x6d
#define KEYCODE_n 0x6e
#define KEYCODE_o 0x6f
#define KEYCODE_p 0x70
#define KEYCODE_q 0x71
#define KEYCODE_r 0x72
#define KEYCODE_s 0x73
#define KEYCODE_t 0x74
#define KEYCODE_u 0x75
#define KEYCODE_v 0x76
#define KEYCODE_w 0x77
#define KEYCODE_x 0x78
#define KEYCODE_y 0x79
#define KEYCODE_z 0x7a
#define KEYCODE_ESCAPE 0x1B

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

class TeleopJointsKeyboard
{
public:
  TeleopJointsKeyboard() : has_recieved_joints_(false), nh_("~")
  {
    std::cout << "init " << std::endl;
    joints_sub_ =
        nh_.subscribe<sensor_msgs::JointState>("/input/joint_states", 1, &TeleopJointsKeyboard::stateCallback, this);
    drive_effort_pub_ = nh_.advertise<std_msgs::Float64>("/output/drive_effort", 1);
    steering_position_pub_ = nh_.advertise<std_msgs::Float64>("/output/steering_position", 1);

    delta = nh_.param<double>("delta", 0.5);
    steer_cmd_.data = 0;
    drive_cmd_.data = 0;
  }

  ~TeleopJointsKeyboard()
  {
  }

  void stateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    if (msg->position.size() != 2)
    {
      ROS_ERROR_STREAM("Not enough joints!");
      exit(-1);
    }

    // Important safety feature
    has_recieved_joints_ = true;
  }

  void keyboardLoop()
  {
    char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use 'W' for throttle");
    puts("Use 'S' for braking");
    puts("Use 'A' for steering left");
    puts("Use 'D' for steering right");
    puts("ESC to end");

    for (;;)
    {
      // get the next event from the keyboard
      if (read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }

      switch (c)
      {
        case KEYCODE_d:
          steer_cmd_.data -= delta;
          break;
        case KEYCODE_a:
          steer_cmd_.data += delta;
          break;
        case KEYCODE_w:
          drive_cmd_.data += delta;
          break;
        case KEYCODE_s:
          drive_cmd_.data -= delta;
          break;
        case KEYCODE_e:
          drive_cmd_.data = 0;
          steer_cmd_.data = 0;
          break;

        case KEYCODE_ESCAPE:
          std::cout << std::endl;
          std::cout << "Exiting " << std::endl;
          quit(0);
          break;

        default:
          std::cout << "CODE: " << c << std::endl;
      }

      // Publish command
      // Important safety feature
      if (!has_recieved_joints_)
      {
        ROS_ERROR_STREAM_NAMED("joint_teleop", "Unable to send joint commands because robot state is invalid");
      }
      else
      {
        std::cout << ".";
        drive_effort_pub_.publish(drive_cmd_);
        steering_position_pub_.publish(steer_cmd_);
      }
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher drive_effort_pub_, steering_position_pub_;
  ros::Subscriber joints_sub_;
  std_msgs::Float64 drive_cmd_, steer_cmd_;
  bool has_recieved_joints_;
  double delta;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joints_teleop_keyboard");
  signal(SIGINT, quit);

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(1);
  spinner.start();

  TeleopJointsKeyboard teleop;
  teleop.keyboardLoop();

  return (0);
}
