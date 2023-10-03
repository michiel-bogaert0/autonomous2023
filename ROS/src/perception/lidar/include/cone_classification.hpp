#ifndef CONECLASSIFICATION_HPP
#define CONECLASSIFICATION_HPP

#include "geometry_msgs/Point32.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>

namespace ns_lidar {
typedef struct {
  geometry_msgs::Point32 pos;
  bool is_cone;
  float color;
  double bounds[3];
  double cone_metric;
} ConeCheck;

typedef struct {
  double x;
  double y;
  double floor;
  double height_cone = 0.325;
  double half_width_cone = 0.114;
} ConeModel;

float hypot3d(float a, float b, float c);

class ConeClassification {

public:
  ConeClassification(ros::NodeHandle &n);

  ConeCheck classifyCone(const pcl::PointCloud<pcl::PointXYZINormal> cloud);

private:
  ros::NodeHandle &n_;

  double point_count_threshold_; // How much % can the cone point count
                                 // prediction be off from the actual count
  int minimal_points_cone_;
  float minimal_height_cone_; // minimal height of cone above the floor
                              // threshold
  float
      threshold_height_big_cone_; // minimal height of cone above the floor
                                  // threshold to be classified as a orange cone
  double cone_shape_factor_; // how similar should a pointcloud be to the cone
                             // model
                             // 0 -> the pointcloud can have any shape
                             // 1 -> the pointcloud must have the precise shape
                             // of a cone
  double height_width_factor_;   // the minimal value height/depth needs to have
  bool use_orange_cones_;        // are orange cones used or not
  bool use_white_cones_;         // are white cones passed forward or not
  double threshold_white_cones_; // distance where the lidar will stop
                                 // determining color and set the color to white

  double first_tipping_distance_; // distance(m) which marks the transition
                                  // between first linear and non-linear region
  double second_tipping_distance_; // distance(m) which marks the transition
                                   // between non-linear region and second linear
                                   // region
  double zero_value_distance_; // distance(m) after which the belief is 0
  double value_start_;         // belief at 0m
  double value_first_tipping_distance_;  // belief at start non-linear region
  double value_second_tipping_distance_; // belief at end non-linear region

  double checkShape(pcl::PointCloud<pcl::PointXYZINormal> cone,
                    Eigen::Vector4f centroid, bool orange);

  double calculateBelief(float dist);
};
} // namespace ns_lidar

#endif