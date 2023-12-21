#ifndef BOUNDARY_ESTIMATION_HPP
#define BOUNDARY_ESTIMATION_HPP

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <ugr_msgs/Boundaries.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>

#include "managed_node.hpp"
#include <color_connector.hpp>
#include <node_fixture/node_fixture.hpp>

namespace boundaryestimation {

class BoundaryEstimation : public node_fixture::ManagedNode {
public:
  explicit BoundaryEstimation(ros::NodeHandle &n);
  void doConfigure() override;
  void doActivate() override;

private:
  ros::NodeHandle n_;

  ColorConnector color_connector_;

  // Publishers
  ros::Publisher boundary_pub_;
  ros::Publisher left_boundary_pub_;
  ros::Publisher right_boundary_pub_;
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

} // namespace boundaryestimation

#endif