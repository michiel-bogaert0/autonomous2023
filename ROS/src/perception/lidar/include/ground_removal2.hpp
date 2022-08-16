#ifndef GROUNDREMOVAL2_HPP
#define GROUNDREMOVAL2_HPP

#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>

namespace ns_lidar {


class GroundRemoval2 {

public:
  GroundRemoval2(ros::NodeHandle &n);

  void groundRemoval2(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points);

private:
 struct {
  bool operator()(pcl::PointXYZI a, pcl::PointXYZI b) const {
    return a.z < b.z;
  }
} zsort;
  ros::NodeHandle &n_;
  double th_floor_;
};
} // namespace ns_lidar

#endif