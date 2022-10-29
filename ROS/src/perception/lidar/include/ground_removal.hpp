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

struct {
  bool operator()(pcl::PointXYZI a, pcl::PointXYZI b) const {
    return a.z < b.z;
  }
} zsort;

class GroundRemoval {

public:
  GroundRemoval(ros::NodeHandle &n);

  void
  groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points,
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground_points);

private:
  ros::NodeHandle &n_;

  std::string ground_removal_method_; // Default: bins, others: zermas

  int num_iter_;    // Number of iterations
  int num_lpr_;     // number of points used to estimate the LPR: Lowest Point
                    // Representative -> a point defined as the average of the
                    // num_lpr_ lowest points
  double th_seeds_; // Threshold for points to be considered initial seeds
  double th_dist_;  // Threshold distance from the plane
  double sensor_height_; // GroundRemoval sensor height to ground (m)

  double th_floor_;     // Threshold distance from the floor level
  int angular_buckets_; // number of angular levels for buckets
  int radial_buckets_;  // number of radial levels for buckets

  void groundRemovalZermas(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points,
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground_points);

  void
  groundRemovalBins(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points,
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground_points);

  model_t estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &seed_points);
  double calculate_ground(pcl::PointXYZ prev_centroid,
                          pcl::PointXYZ current_centroid,
                          pcl::PointXYZ next_centroid, pcl::PointXYZI point,
                          int position);
  void
  extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points);

  void
  process_bucket(pcl::PointCloud<pcl::PointXYZI> bucket,
                 pcl::PointXYZ prev_centroid, pcl::PointXYZ current_centroid,
                 pcl::PointXYZ next_centroid, int position,
                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points,
                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground_points);
};
} // namespace ns_lidar

#endif
