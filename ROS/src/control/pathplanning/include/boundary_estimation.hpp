#ifndef BOUNDARY_ESTIMATION_HPP
#define BOUNDARY_ESTIMATION_HPP

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <node_fixture/node_fixture.hpp>
#include <pathplanning.hpp>
#include <triangulator.hpp>

namespace pathplanning {

class BoundaryEstimation {
public:
  explicit BoundaryEstimation(ros::NodeHandle &n);

private:
  ros::NodeHandle n_;

  TransformFrames frametf_;

  // Publishers
  ros::Publisher path_pub_;
  ros::Publisher path_stamped_pub_;

  // Diagnostic publisher
  std::unique_ptr<node_fixture::DiagnosticPublisher> diagnostics_pub;

  // Subscribers
  ros::Subscriber map_sub_;

  void receive_new_map(
      const ugr_msgs::ObservationWithCovarianceArrayStamped::ConstPtr &track);
  void compute(const std::vector<std::vector<double>> &cones,
               const std_msgs::Header &header);
};

} // namespace pathplanning

#endif