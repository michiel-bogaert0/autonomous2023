#include "boundary_estimation.hpp"

namespace pathplanning {

BoundaryEstimation::BoundaryEstimation(ros::NodeHandle &n)
    : n_(n), frametf_(n) {
  this->path_pub_ = n_.advertise<nav_msgs::Path>("/output/boundaries", 10);
  this->map_sub_ = n_.subscribe("/input/local_map", 10,
                                &BoundaryEstimation::receive_new_map, this);
  this->diagnostics_pub = std::unique_ptr<node_fixture::DiagnosticPublisher>(
      new node_fixture::DiagnosticPublisher(n, "CTRL BOUNDARY"));
}

void BoundaryEstimation::receive_new_map(
    const ugr_msgs::ObservationWithCovarianceArrayStamped::ConstPtr &track) {
  std::vector<std::vector<double>> cones;
  for (const auto &obs_with_cov : track->observations) {
    std::vector<double> cone;
    cone.push_back(obs_with_cov.observation.location.x);
    cone.push_back(obs_with_cov.observation.location.y);
    cone.push_back(obs_with_cov.observation.observation_class);
    cones.push_back(cone);
  }

  compute(cones, track->header);
}

void BoundaryEstimation::compute(const std::vector<std::vector<double>> &cones,
                                 const std_msgs::Header &header) {}

} // namespace pathplanning