#ifndef CONECLUSTERING_HPP
#define CONECLUSTERING_HPP

#include "cone_classification.hpp"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>

namespace ns_lidar {
struct {
  bool operator()(pcl::PointXYZINormal a, pcl::PointXYZINormal b) const {
    return a.y < b.y;
  }
} leftrightsort;

class ConeClustering {

public:
  ConeClustering(ros::NodeHandle &n);

  std::vector<pcl::PointCloud<pcl::PointXYZINormal>>
  cluster(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud,
          const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &ground);
  sensor_msgs::PointCloud
      constructMessage(std::vector<pcl::PointCloud<pcl::PointXYZINormal>>);
  sensor_msgs::PointCloud2 clustersColoredMessage(std::vector<pcl::PointCloud<pcl::PointXYZINormal>>);

private:
  ros::NodeHandle &n_;

  ConeClassification coneClassification_;

  std::string clustering_method_; // Default: euclidian, others: string
  double cluster_tolerance_;      // The cone clustering tolerance (m)
  double
      min_distance_factor_; // distance around the cone that contains no
                            // other cones as a factor to the width of the cone

  std::vector<pcl::PointCloud<pcl::PointXYZINormal>>
  euclidianClustering(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud,
                      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &ground);
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>>
  stringClustering(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud);
  float arctan(float x, float y);
};
} // namespace ns_lidar

#endif