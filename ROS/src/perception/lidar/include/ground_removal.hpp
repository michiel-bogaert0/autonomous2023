#ifndef GROUNDREMOVAL_HPP
#define GROUNDREMOVAL_HPP

#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace ns_lidar {

enum PositionEnum { BEGIN, MID, END };

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
  explicit GroundRemoval(ros::NodeHandle &n);

  void
  groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points,
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground_points);
  sensor_msgs::PointCloud2
  publishColoredGround(pcl::PointCloud<pcl::PointXYZINormal> points,
                       const sensor_msgs::PointCloud2 &msg);

private:
  ros::NodeHandle &n_;

  std::string ground_removal_method_; // Default: bins, others: zermas

  double min_distance_; // Preprocessing parameters
  double max_distance_; //
  double min_angle_;    //
  double max_angle_;    //

  int num_iter_;    // Number of iterations
  int num_lpr_;     // number of points used to estimate the LPR: Lowest Point
                    // Representative -> a point defined as the average of the
                    // num_lpr_ lowest points
  double th_seeds_; // Threshold for points to be considered initial seeds
  double th_dist_;  // Threshold distance from the plane
  double sensor_height_; // GroundRemoval sensor height to ground (m)
  double th_floor_;      // Threshold distance from the floor level
  int angular_buckets_;  // number of angular levels for buckets
  double radial_bucket_tipping_point_; // edge between small buckets and big
                                       // buckets
  double small_radial_bucket_length_;  // length of small buckets
  double big_radial_bucket_length_;    // length of big buckets
  bool use_slope_;                     // Is the slope used for GR or not

  bool noisy_environment_;    // whether the lidar is used in a busy/noisy room
                              // (more points)
  double max_bucket_height_;  // maximum height difference between top points
                              // and lower points in bin for noisy environments
  int min_points_per_bucket_; // minimum points allowed in a bucket for noisy
                              // environments
  int max_points_per_bucket_; // maximum points allowed in a bucket for noisy
                              // environments
  int factor_color_; // number of consecutive points that get the same color

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
                          PositionEnum position);
  void
  extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points);

  void
  process_bucket(pcl::PointCloud<pcl::PointXYZI> bucket,
                 pcl::PointXYZ prev_centroid, pcl::PointXYZ current_centroid,
                 pcl::PointXYZ next_centroid, PositionEnum position,
                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points,
                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground_points);

  struct {
    bool operator()(pcl::PointXYZINormal a, pcl::PointXYZINormal b) const {
      return a.y > b.y;
    }
  } leftrightsort;
};
} // namespace ns_lidar

#endif
