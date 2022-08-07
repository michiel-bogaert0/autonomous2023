#ifndef GROUNDREMOVAL_HPP
#define GROUNDREMOVAL_HPP

#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>

namespace ns_lidar {

typedef struct {
  Eigen::MatrixXf normal_n;
  double d = 0.;
} model_t;

class GroundRemoval {

public:
  GroundRemoval(ros::NodeHandle &n);

  void groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points);

private:
  ros::NodeHandle &n_;

  int num_iter_;    // Number of iterations
  int num_lpr_;     // number of points used to estimate the LPR: Lowest Point
                    // Representative -> a point defined as the average of the
                    // num_lpr_ lowest points
  double th_seeds_; // Threshold for points to be considered initial seeds
  double th_dist_;  // Threshold distance from the plane
  double sensor_height_; // GroundRemoval sensor height to ground (m)

  model_t estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &seed_points);
  void
  extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points);
};
} // namespace ns_lidar

#endif
