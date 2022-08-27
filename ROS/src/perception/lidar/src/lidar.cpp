#include "lidar.hpp"
#include <chrono>

#include <ugr_msgs/Observations.h>
#include <ugr_msgs/Observation.h>

// Constructor
namespace ns_lidar
{
  Lidar::Lidar(ros::NodeHandle &n)
      : n_(n), cone_clustering_(n), ground_removal_(n)
  {
    // Subscribe to the raw lidar topic
    // rawLidarSubscriber_ = n.subscribe("perception/raw_pc", 10,
    // &Lidar::rawPcCallback, this);
    rawLidarSubscriber_ =
        n.subscribe("/os_cloud_node/points", 10, &Lidar::rawPcCallback, this);

    // Publish to the filtered and clustered lidar topic
    preprocessedLidarPublisher_ =
        n.advertise<sensor_msgs::PointCloud2>("perception/preprocessed_pc", 5);
    groundRemovalLidarPublisher_ =
        n.advertise<sensor_msgs::PointCloud2>("perception/groundremoval_pc", 5);
    clusteredLidarPublisher_ =
        n.advertise<sensor_msgs::PointCloud>("perception/clustered_pc", 5);
    visPublisher_ =
        n.advertise<visualization_msgs::MarkerArray>("perception/cones_lidar", 5);
    conePublisher_ =
        n.advertise<ugr_msgs::Observations>("perception/observations", 5);

    n.param<bool>("show_debug", show_debug_, false);
  }

  /**
   * @brief Subscribes to the LIDAR raw pointcloud topic and processes the data
   *
   * @arg msg: the PointCloud2 message
   */
  void Lidar::rawPcCallback(const sensor_msgs::PointCloud2 &msg)
  {
    // Create PC objects
    pcl::PointCloud<pcl::PointXYZI> raw_pc_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr preprocessed_pc(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, raw_pc_);
    if(show_debug_) ROS_INFO("Raw points: %ld", raw_pc_.size());

    // Preprocessing
    preprocessing(raw_pc_, preprocessed_pc);
    if(show_debug_) ROS_INFO("Preprocessed points: %ld", preprocessed_pc->size());

    sensor_msgs::PointCloud2 preprocessed_msg;
    pcl::toROSMsg(*preprocessed_pc, preprocessed_msg);
    preprocessed_msg.header.stamp = msg.header.stamp;
    preprocessed_msg.header.frame_id = msg.header.frame_id;
    preprocessedLidarPublisher_.publish(preprocessed_msg);

    // Ground plane removal
    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(
        new pcl::PointCloud<pcl::PointXYZI>());
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    ground_removal_.groundRemoval(preprocessed_pc, notground_points,
                                  ground_points);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    if(show_debug_) ROS_INFO("Post ground removal points: %ld", notground_points->size());
    double time_round =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2)
            .count();
    if(show_debug_) ROS_INFO("ground-removal took: %lf", time_round);

    sensor_msgs::PointCloud2 groundremoval_msg;
    pcl::toROSMsg(*notground_points, groundremoval_msg);
    groundremoval_msg.header.frame_id = msg.header.frame_id;
    groundremoval_msg.header.stamp = msg.header.stamp;
    groundRemovalLidarPublisher_.publish(groundremoval_msg);

    // Cone clustering
    sensor_msgs::PointCloud cluster;

    t2 = std::chrono::steady_clock::now();
    cluster = cone_clustering_.cluster(notground_points, ground_points);
    t1 = std::chrono::steady_clock::now();
    time_round =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2)
            .count();
    if(show_debug_) ROS_INFO("Clustering took: %lf", time_round);

    cluster.header.frame_id = msg.header.frame_id;
    cluster.header.stamp = msg.header.stamp;
    if(show_debug_) ROS_INFO("Clustered points: %ld", cluster.points.size());

    clusteredLidarPublisher_.publish(cluster);

    // Create an array of markers to display in Foxglove
    publishMarkers(cluster);
    publishObservations(cluster);
  }

  /**
   * @brief Preprocesses the given raw point cloud.
   *
   * Cleans up points containing the car itself and does ground plane removal.
   *
   * @arg raw: the raw PC
   * @arg preprocessed_pc: the resulting filtered PC
   *
   */
  void Lidar::preprocessing(
      const pcl::PointCloud<pcl::PointXYZI> &raw,
      pcl::PointCloud<pcl::PointXYZI>::Ptr &preprocessed_pc)
  {
    // Clean up the points belonging to the car and noise in the sky
    for (auto &iter : raw.points)
    {
      // Remove points closer than 1m, higher than 0.6m or further than 20m
      if (std::hypot(iter.x, iter.y) < 1 || iter.z > 1 ||
          std::hypot(iter.x, iter.y) > 21 || std::atan2(iter.x, iter.y) < 0.3 || std::atan2(iter.x, iter.y) > 2.8)
        continue;

      preprocessed_pc->points.push_back(iter);
    }
  }

  /**
   * @brief Publishes the Lidar observations for real
   *
   * @param cones
   */
  void Lidar::publishObservations(const sensor_msgs::PointCloud cones)
  {
    ugr_msgs::Observations observations;
    observations.header.frame_id = cones.header.frame_id;
    observations.header.stamp = cones.header.stamp;

    int i = 0;
    for (auto cone : cones.points)
    {
      ugr_msgs::Observation observation;

      // If color == 0, then it is a BLUE cone, and Cones.BLUE in fs_msgs/Cone is 0
      // color == 1 is yellow
      float color = cones.channels[0].values[i];
      observation.observation_class = color;

      observation.location.x = cone.x;
      observation.location.y = cone.y;
      observation.location.z = cone.z;

      observations.observations.push_back(observation);

      i++;
    }

    conePublisher_.publish(observations);
  }

  /**
   * @brief Publishes a MarkerArray for visualisation purposes
   *
   */
  void Lidar::publishMarkers(const sensor_msgs::PointCloud cones)
  {
    visualization_msgs::MarkerArray markers;

    int i = 0;
    for (auto cone : cones.points)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = cones.header.frame_id;
      marker.header.stamp = cones.header.stamp;
      marker.ns = "cones";
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      float color = cones.channels[0].values[i];
      float x_size = cones.channels[1].values[i];
      float y_size = cones.channels[2].values[i];
      float z_size = cones.channels[3].values[i];
      marker.id = i++;

      marker.pose.position.x = cone.x;
      marker.pose.position.y = cone.y;
      marker.pose.position.z = cone.z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.5; // 0.228; // in meters
      marker.scale.y = 0.5; // 0.228;
      marker.scale.z = 0.5; // 0.325;

      marker.color.r = color;
      marker.color.g = color;
      marker.color.b = 1 - color;
      marker.color.a = 1;

      markers.markers.push_back(marker);
    }

    visualization_msgs::MarkerArray clear_cones;
    visualization_msgs::Marker marker;
    marker.header.frame_id = cones.header.frame_id;
    marker.header.stamp = cones.header.stamp;
    marker.ns = "cones";
    marker.action = visualization_msgs::Marker::DELETEALL;
    clear_cones.markers.push_back(marker);
    visPublisher_.publish(clear_cones);

    visPublisher_.publish(markers);
  }
} // namespace ns_lidar
