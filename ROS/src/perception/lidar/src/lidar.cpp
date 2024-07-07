#include "lidar.hpp"
#include "diagnostic_msgs/DiagnosticArray.h"
#include <chrono>
#include <fstream>
#include <iostream>

// Constructor
namespace ns_lidar {
Lidar::Lidar(ros::NodeHandle &n)
    : n_(n), cone_clustering_(n), ground_removal_(n) {
  // Subscribe to the raw lidar topic
  // rawLidarSubscriber_ = n.subscribe("perception/raw_pc", 10,
  // &Lidar::rawPcCallback, this);
  rawLidarSubscriber_ =
      n.subscribe("/ugr/car/sensors/lidar", 10, &Lidar::rawPcCallback, this);

  n.param<bool>("publish_preprocessing", publish_preprocessing_, false);
  n.param<bool>("publish_ground", publish_ground_, false);
  n.param<bool>("publish_clusters", publish_clusters_, true);
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
      n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 5);
}

/**
 * @brief Subscribes to the LIDAR raw pointcloud topic and processes the data
 *
 * @arg msg: the PointCloud2 message
 */
void Lidar::rawPcCallback(const sensor_msgs::PointCloud2 &msg) {
  // Create PC objects
  pcl::PointCloud<pcl::PointXYZI> raw_pc_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr preprocessed_pc(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(msg, raw_pc_);
  publishDiagnostic(OK, "[perception] raw points",
                    "#points: " + std::to_string(raw_pc_.size()));
  // flip pointcloud if the lidar is rotated
  if (lidar_rotated_) {
    raw_pc_ = flipPointcloud(raw_pc_);
  }
  // Preprocessing
  preprocessing(raw_pc_, preprocessed_pc);
  publishDiagnostic(OK, "[perception] preprocessed points",
                    "#points: " + std::to_string(preprocessed_pc->size()));

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

  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  ground_removal_.groundRemoval(preprocessed_pc, notground_points,
                                ground_points);
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  publishDiagnostic(OK, "[perception] ground removal points",
                    "#points: " + std::to_string(notground_points->size()));

  double time_round =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2)
          .count();
  publishDiagnostic(time_round < 1 ? OK : WARN,
                    "[perception] ground removal time",
                    "time needed: " + std::to_string(time_round));

  if (publish_ground_) {
    sensor_msgs::PointCloud2 groundremoval_msg;
    pcl::toROSMsg(*notground_points, groundremoval_msg);
    groundremoval_msg.header.frame_id = msg.header.frame_id;
    groundremoval_msg.header.stamp = msg.header.stamp;
    groundRemovalLidarPublisher_.publish(groundremoval_msg);

    // publish colored pointcloud to check order points
    sensor_msgs::PointCloud2 ground_msg =
        ground_removal_.publishColoredGround(*notground_points, msg);
    groundColoredPublisher_.publish(ground_msg);
  }

  // Cone clustering
  sensor_msgs::PointCloud cluster;
  sensor_msgs::PointCloud2 clustersColored;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>> clusters;

  t2 = std::chrono::steady_clock::now();
  clusters = cone_clustering_.cluster(notground_points, ground_points);
  cluster = cone_clustering_.constructMessage(clusters);
  t1 = std::chrono::steady_clock::now();
  time_round =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2)
          .count();
  publishDiagnostic(time_round < 1 ? OK : WARN, "[perception] clustering time",
                    "time needed: " + std::to_string(time_round));

  if (publish_clusters_) {
    clustersColored = cone_clustering_.clustersColoredMessage(clusters);
    clustersColored.header.frame_id = msg.header.frame_id;
    clustersColored.header.stamp = msg.header.stamp;
    clustersColoredpublisher_.publish(clustersColored);
  }

  cluster.header.frame_id = msg.header.frame_id;
  cluster.header.stamp = msg.header.stamp;
  clusteredLidarPublisher_.publish(cluster);
  publishDiagnostic(OK, "[perception] clustering points",
                    "#points: " + std::to_string(cluster.points.size()));

  // Create an array of markers to display in Foxglove
  publishObservations(cluster);
  publishDiagnostic(OK, "[perception] end processing", "pointcloud processed");
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
    if (std::hypot(iter.x, iter.y) < 1 || iter.z > -1 ||
        std::hypot(iter.x, iter.y) > 21 || std::atan2(iter.x, iter.y) < 0.3 ||
        std::atan2(iter.x, iter.y) > 2.8)
      continue;
    preprocessed_pc->points.push_back(iter);
  }
}

/**
 * @brief Publishes the Lidar observations for real
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
                              std::string message) {
  diagnostic_msgs::DiagnosticArray diag_array;
  diagnostic_msgs::DiagnosticStatus diag_status;
  diag_status.level = status;
  diag_status.name = name;
  diag_status.message = message;
  diag_array.status.push_back(diag_status);

  diagnosticPublisher_.publish(diag_array);
}

template <class PointT>
pcl::PointCloud<PointT> Lidar::flipPointcloud(pcl::PointCloud<PointT> pc) {
  pcl::PointCloud<PointT> *new_pc = new pcl::PointCloud<PointT>;
  for (auto &iter : pc.points) {
    PointT new_point = *new PointT;
    new_point.x = iter.x;
    new_point.y = -iter.y;
    new_point.z = -iter.z;
    new_pc->points.push_back(new_point);
  }
  return *new_pc;
}
} // namespace ns_lidar
