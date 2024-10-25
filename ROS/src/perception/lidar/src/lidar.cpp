#include "lidar.hpp"
#include "diagnostic_msgs/DiagnosticArray.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <tuple>

// Constructor
namespace ns_lidar {
Lidar::Lidar(ros::NodeHandle &n)
    : n_(n), cone_clustering_(n), ground_removal_(n) {

  // Set parameters for preprocessing
  n.param<double>("min_distance", min_distance_, 1.0);
  n.param<double>("max_distance", max_distance_, 21.0);
  n.param<double>("max_height", max_height_, 0.5);
  n.param<double>("sensor_height", sensor_height_, 0.98);
  n.param<double>("min_angle", min_angle_, 0.3);
  n.param<double>("max_angle", max_angle_, 2.8);

  // Subscribe to the raw lidar topic
  rawLidarSubscriber_ =
      n.subscribe("/ugr/car/sensors/lidar", 10, &Lidar::rawPcCallback, this);

  n.param<bool>("publish_diagnostics", publish_diagnostics_, true);
  n.param<bool>("publish_diagnostics", publish_clusters_, true);
  n.param<bool>("publish_preprocessing", publish_preprocessing_, false);
  n.param<bool>("publish_ground", publish_ground_, false);
  n.param<bool>("publish_clusters", publish_clusters_, false);

  n.param<bool>("lidar_rotated", lidar_rotated_, false);
  // Publish to the filtered and clustered lidar topic
  if (publish_preprocessing_)
    preprocessedLidarPublisher_ =
        n.advertise<sensor_msgs::PointCloud2>("perception/preprocessed_pc", 5);
  if (publish_ground_) {
    groundRemovalLidarPublisher_ =
        n.advertise<sensor_msgs::PointCloud2>("perception/groundremoval_pc", 5);
    groundColoredPublisher_ =
        n.advertise<sensor_msgs::PointCloud2>("/perception/ground_colored", 5);
  }

  clusteredLidarPublisher_ =
      n.advertise<sensor_msgs::PointCloud>("perception/clustered_pc", 5);
  if (publish_clusters_)
    clustersColoredpublisher_ =
        n.advertise<sensor_msgs::PointCloud2>("perception/clusters_colored", 5);

  conePublisher_ = n.advertise<ugr_msgs::ObservationWithCovarianceArrayStamped>(
      "perception/observations", 5);
  diagnosticPublisher_ =
      n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics/lidar", 5);
}

/**
 * @brief Subscribes to the LIDAR raw pointcloud topic and processes the data
 *
 * @arg msg: the PointCloud2 message
 */
void Lidar::rawPcCallback(const sensor_msgs::PointCloud2 &msg) {
  std::chrono::steady_clock::time_point t_start =
      std::chrono::steady_clock::now();

  // Create PC objects
  pcl::PointCloud<pcl::PointXYZI> raw_pc_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr preprocessed_pc(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(msg, raw_pc_);

  // flip pointcloud if the lidar is rotated
  if (lidar_rotated_) {
    raw_pc_ = flipPointcloud(raw_pc_);
  }
  // Preprocessing
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  preprocessing(raw_pc_, preprocessed_pc);
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  latency_preprocessing_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
          .count();

  if (publish_preprocessing_) {
    sensor_msgs::PointCloud2 preprocessed_msg;
    pcl::toROSMsg(*preprocessed_pc, preprocessed_msg);
    preprocessed_msg.header.stamp = msg.header.stamp;
    preprocessed_msg.header.frame_id = msg.header.frame_id;
    preprocessedLidarPublisher_.publish(preprocessed_msg);
  }

  // Ground plane removal
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points(
      new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground_points(
      new pcl::PointCloud<pcl::PointXYZINormal>());

  t0 = std::chrono::steady_clock::now();
  ground_removal_.groundRemoval(preprocessed_pc, notground_points,
                                ground_points);
  t1 = std::chrono::steady_clock::now();

  latency_ground_removal_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
          .count();

  if (publish_ground_) {
    // Create a copy of notground_points for publishing
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points_copy(
        new pcl::PointCloud<pcl::PointXYZINormal>(*notground_points));
    if (lidar_rotated_) {
      *notground_points_copy = flipPointcloud(*notground_points_copy);
    }

    sensor_msgs::PointCloud2 groundremoval_msg;
    pcl::toROSMsg(*notground_points_copy, groundremoval_msg);
    groundremoval_msg.header.frame_id = msg.header.frame_id;
    groundremoval_msg.header.stamp = msg.header.stamp;
    groundRemovalLidarPublisher_.publish(groundremoval_msg);

    // Publish colored pointcloud to check order points
    sensor_msgs::PointCloud2 ground_msg =
        ground_removal_.publishColoredGround(*notground_points_copy, msg);
    groundColoredPublisher_.publish(ground_msg);
  }

  // Cone clustering
  sensor_msgs::PointCloud cluster;
  sensor_msgs::PointCloud2 clustersColored;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>> clusters;
  std::tuple<sensor_msgs::PointCloud,
             std::vector<pcl::PointCloud<pcl::PointXYZINormal>>>
      msg_and_coneclusters;

  t0 = std::chrono::steady_clock::now();
  clusters = cone_clustering_.cluster(notground_points, ground_points);
  msg_and_coneclusters = cone_clustering_.constructMessage(clusters);
  cluster = std::get<0>(msg_and_coneclusters);
  t1 = std::chrono::steady_clock::now();
  latency_clustering_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
          .count();

  if (publish_clusters_) {
    clustersColored = cone_clustering_.clustersColoredMessage(clusters);
    clustersColored.header.frame_id = msg.header.frame_id;
    clustersColored.header.stamp = msg.header.stamp;
    clustersColoredpublisher_.publish(clustersColored);
  }

  cluster.header.frame_id = msg.header.frame_id;
  cluster.header.stamp = msg.header.stamp;
  clusteredLidarPublisher_.publish(cluster);

  publishObservations(cluster);

  std::chrono::steady_clock::time_point t_end =
      std::chrono::steady_clock::now();
  latency_total_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start)
          .count();

  if (publish_diagnostics_) {
    // Raw point cloud diagnostics
    publishDiagnostic(OK, "[LIDAR] Raw pointcloud", "", "#Points",
                      std::to_string(raw_pc_.size()));

    // Preprocessing diagnostics
    publishDiagnostic(latency_preprocessing_ < 1 ? OK : WARN,
                      "[LIDAR] Preprocessing",
                      std::to_string(latency_preprocessing_), "#Points",
                      std::to_string(preprocessed_pc->size()));

    // Ground removal diagnostics
    publishDiagnostic(
        latency_ground_removal_ < 1 ? OK : WARN, "[LIDAR] Ground removal",
        std::to_string(latency_ground_removal_), "#Non-ground points",
        std::to_string(notground_points->size()));

    // Points per cone
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>> cone_clusters;
    cone_clusters = std::get<1>(msg_and_coneclusters);
    if (cone_clusters.size() == cluster.points.size()) {
      for (int i = 0; i < cone_clusters.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZINormal> &cone_cluster = cone_clusters[i];
        auto cone = cluster.points[i];
        sensor_msgs::PointCloud cone_msg;

        diagnostic_msgs::DiagnosticArray diag_array;
        diagnostic_msgs::DiagnosticStatus diag_status;
        diag_status.level = OK;
        diag_status.name = "[LIDAR] Points per cone";

        diagnostic_msgs::KeyValue distance;
        diagnostic_msgs::KeyValue numpoints;
        distance.key = "distance";
        distance.value = std::to_string(hypot3d(cone.x, cone.y, cone.z));
        diag_status.values.push_back(distance);
        numpoints.key = "numpoints";
        numpoints.value = std::to_string(cone_cluster.size());
        diag_status.values.push_back(numpoints);

        diag_array.status.push_back(diag_status);
        diagnosticPublisher_.publish(diag_array);
      }
    }

    // Clustering diagnostics
    publishDiagnostic(latency_clustering_ < 1 ? OK : WARN, "[LIDAR] Clustering",
                      std::to_string(latency_clustering_), "#Cone clusters",
                      std::to_string(clusters.size()));

    // Total lidar pipeline latency
    publishDiagnostic(latency_total_ < 1 ? OK : WARN, "[LIDAR] End processing",
                      std::to_string(latency_total_));
  }
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr &preprocessed_pc) {
  // Clean up the points belonging to the car and noise in the sky
  for (auto &iter : raw.points) {
    // Remove points closer than 1m, higher than 0.5m or further than 20m
    // and points outside the frame of Pegasus
    if (std::hypot(iter.x, iter.y) < min_distance_ || iter.z > -0.1 ||
        //-sensor_height_ + max_height_ ||
        std::hypot(iter.x, iter.y) > max_distance_ ||
        std::atan2(iter.x, iter.y) < min_angle_ ||
        std::atan2(iter.x, iter.y) > max_angle_) {
      continue;
    }
    preprocessed_pc->points.push_back(iter);
  }
}

/**
 * @brief Classifies cone clusters and publishes the Lidar observations
 *
 * @param cones
 */
void Lidar::publishObservations(const sensor_msgs::PointCloud cones) {
  ugr_msgs::ObservationWithCovarianceArrayStamped observations;
  observations.header.frame_id = "ugr/car_base_link/os_sensor_normal";
  observations.header.stamp = cones.header.stamp;

  int i = 0;
  for (auto cone : cones.points) {
    ugr_msgs::ObservationWithCovariance observationWithCovariance;

    // If color == 0, then it is a BLUE cone, and Cones.BLUE in fs_msgs/Cone is
    // 0 color == 1 is yellow
    float color = cones.channels[0].values[i];
    observationWithCovariance.observation.observation_class = color;

    observationWithCovariance.observation.location.x = cone.x;
    observationWithCovariance.observation.location.y = cone.y;
    observationWithCovariance.observation.location.z = cone.z;
    observationWithCovariance.covariance =
        boost::array<double, 9>({0.2, 0, 0, 0, 0.2, 0, 0, 0, 0.8});

    double cone_metric = cones.channels[4].values[i];
    observationWithCovariance.observation.belief = cone_metric;

    observations.observations.push_back(observationWithCovariance);

    i++;
  }

  conePublisher_.publish(observations);
}

void Lidar::publishDiagnostic(DiagnosticStatusEnum status, std::string name,
                              std::string latency, std::string key,
                              std::string value) {
  diagnostic_msgs::DiagnosticArray diag_array;
  diagnostic_msgs::DiagnosticStatus diag_status;
  diagnostic_msgs::KeyValue latency_keyval;
  diagnostic_msgs::KeyValue keyval;

  latency_keyval.key = "latency";
  latency_keyval.value = latency;
  keyval.key = key;
  keyval.value = value;

  diag_status.level = status;
  diag_status.name = name;
  diag_status.values.push_back(latency_keyval);
  diag_status.values.push_back(keyval);
  diag_array.status.push_back(diag_status);

  diagnosticPublisher_.publish(diag_array);
}

template <class PointT>
pcl::PointCloud<PointT> Lidar::flipPointcloud(pcl::PointCloud<PointT> pc) {
  pcl::PointCloud<PointT> *new_pc = new pcl::PointCloud<PointT>;
  for (const auto &iter : pc.points) {
    PointT new_point = *new PointT;
    new_point.x = iter.x;
    new_point.y = -iter.y;
    new_point.z = -iter.z;
    new_point.intensity = iter.intensity;
    new_pc->points.push_back(new_point);
  }
  return *new_pc;
}
} // namespace ns_lidar
