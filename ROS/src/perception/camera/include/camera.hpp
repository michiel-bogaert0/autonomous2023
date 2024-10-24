#ifndef CAMERA_HPP
#define CAMERA_HPP
#define PY_SSIZE_T_CLEAN
#include "neoapi/neoapi.hpp"
#include <boost/range/algorithm.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <yaml-cpp/yaml.h>

class CameraNode {
public:
  explicit CameraNode(ros::NodeHandle &n);

private:
  bool use_raw_;
  bool cam_rotated_;
  std::string camsettings_location_;
  std::string camcal_location_;
  int camera_height_;
  int camera_width_;
  YAML::Node camsettings_;
  double estimated_latency_;
  int rate_value_;
  ros::Subscriber raw_sub_;
  ros::Publisher image_pub_;
  ros::Publisher info_pub_;
  ros::Subscriber sim_sub_;
  ros::Rate rate_{ros::Rate(10)};
  std::string sensor_name_;
  std::string frame_;
  NeoAPI::Cam camera_;
  cv::Mat *camera_matrix_ = new cv::Mat;
  cv::Mat *distortion_matrix_ = new cv::Mat;
  cv::Mat *optimal_camera_matrix_ = new cv::Mat;
  cv::Mat *map1_ = new cv::Mat;
  cv::Mat *map2_ = new cv::Mat;

  sensor_msgs::CameraInfo get_camera_info();
  void setup_camera();
  sensor_msgs::Image process_data();
  std_msgs::Header create_header();
  sensor_msgs::Image cv_to_ros_image(cv::Mat &mat);
  void publish_sub_data(const sensor_msgs::Image &msg);
  void publish_image_data();
  template <int Size> boost::array<double, Size> mat_to_boost(cv::Mat &mat);
};

#endif
