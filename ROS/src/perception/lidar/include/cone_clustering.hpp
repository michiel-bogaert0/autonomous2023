#ifndef CONECLUSTERING_HPP
#define CONECLUSTERING_HPP

#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>

namespace ns_lidar {
typedef struct {
  geometry_msgs::Point32 pos;
  bool is_cone;
  float color;
  double bounds[3];
} ConeCheck;

typedef struct{
  double x;
  double y;
  double floor;
  double height_cone = 0.325;
  double half_width_cone = 0.114;
} ConeModel;

struct {
  bool operator()(pcl::PointXYZINormal a, pcl::PointXYZINormal b) const {
    return a.y < b.y;
  }
} leftrightsort;

class ConeClustering {

public:
  ConeClustering(ros::NodeHandle &n);

  sensor_msgs::PointCloud
  cluster(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud,
          const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &ground);

private:
  ros::NodeHandle &n_;

  std::string clustering_method_; // Default: euclidian, others: string
  double cluster_tolerance_;      // The cone clustering tolerance (m)
  double point_count_threshold_;   // How much % can the cone point count
                                  // prediction be off from the actual count
  double min_distance_factor_; // distance around the cone that contains no
                            // other cones as a factor to the width of the cone
  int minimal_points_cone_;
  float minimal_height_cone_; //minimal height of cone above the floor threshold
  double cone_shape_factor_; //how similar should a pointcloud be to the cone model

  sensor_msgs::PointCloud
  euclidianClustering(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud,
                      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &ground);
  sensor_msgs::PointCloud
  stringClustering(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud);
  sensor_msgs::PointCloud
      construct_message(std::vector<pcl::PointCloud<pcl::PointXYZINormal>>);
  ConeCheck isCloudCone(pcl::PointCloud<pcl::PointXYZINormal> cone);
  bool checkShape(pcl::PointCloud<pcl::PointXYZINormal> cone, Eigen::Vector4f centroid, bool orange);
  float hypot3d(float a, float b, float c);
  float arctan(float x, float y);
};
} // namespace ns_lidar

#endif
