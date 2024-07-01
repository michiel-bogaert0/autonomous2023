#ifndef LIDAR_HPP
#define LIDAR_HPP

#include "cone_clustering.hpp"
#include "ground_removal.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ugr_msgs/Observation.h>
#include <ugr_msgs/ObservationWithCovariance.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>

namespace ns_lidar {

enum DiagnosticStatusEnum { OK, WARN, ERROR, STALE };

class Lidar {

public:
  explicit Lidar(ros::NodeHandle &n);

private:
  ros::NodeHandle n_;
  ros::Subscriber rawLidarSubscriber_;
  ros::Publisher preprocessedLidarPublisher_;
  ros::Publisher groundRemovalLidarPublisher_;
  ros::Publisher clusteredLidarPublisher_;
  ros::Publisher clustersColoredpublisher_;
  ros::Publisher visPublisher_;
  ros::Publisher conePublisher_;
  ros::Publisher diagnosticPublisher_;
  ros::Publisher groundColoredPublisher_;

  ConeClustering cone_clustering_;
  GroundRemoval ground_removal_;

  bool publish_preprocessing_; // publish the preprocessed pointcloud
  bool publish_ground_;        // publish the debug ground pointclouds
  bool publish_clusters_;      // color the clusters and publish them
  std::string blue_url_ = "https://storage.googleapis.com/"
                          "learnmakeshare_cdn_public/blue_cone_final.dae";
  std::string yellow_url_ = "https://storage.googleapis.com/"
                            "learnmakeshare_cdn_public/yellow_cone_final.dae";

  void rawPcCallback(const sensor_msgs::PointCloud2 &msg);
  void preprocessing(const pcl::PointCloud<pcl::PointXYZI> &raw,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr &preprocessed_pc);
  void publishMarkers(const sensor_msgs::PointCloud cones);

  void publishObservations(const sensor_msgs::PointCloud cones);
  void publishDiagnostic(DiagnosticStatusEnum status, std::string name,
                         std::string message);
  template <class PointT>
  pcl::PointCloud<PointT> flipPointcloud(pcl::PointCloud<PointT> pc);
};
} // namespace ns_lidar

#endif
