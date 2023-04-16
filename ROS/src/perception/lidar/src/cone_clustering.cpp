#include "cone_clustering.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#define WIDTH_BIG_CONE 0.285

namespace ns_lidar {
// Constructor
ConeClustering::ConeClustering(ros::NodeHandle &n)
    : n_(n), coneClassification_(n) {
  // Get parameters
  n.param<std::string>("clustering_method", clustering_method_, "string");
  n.param<double>("cluster_tolerance", cluster_tolerance_, 0.4);
  n.param<double>("min_distance_factor", min_distance_factor_, 1.5);
  n.param<double>("cone_reconstruction_treshold", cone_reconstruction_treshold_,
                  0.2);
  n.param<double>("maximal_delta_arc_cluster", max_arc_cluster_, 0.3);
}

/**
 * @brief Clusters the cones in the final filtered point cloud and generates a
 * ROS message.
 *
 * The type of clustering that is applied is chosen by the clustering_method
 * parameter.
 */
std::vector<pcl::PointCloud<pcl::PointXYZINormal>> ConeClustering::cluster(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &ground) {
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>> cluster_msg;
  if (clustering_method_ == "string") {
    cluster_msg = ConeClustering::stringClustering(cloud, ground);
  } else {
    cluster_msg = ConeClustering::euclidianClustering(cloud, ground);
  }

  return cluster_msg;
}

/**
 * @brief Clusters the cones in the final filtered point cloud and returns a
 * vector containing the different clusters
 *
 */
std::vector<pcl::PointCloud<pcl::PointXYZINormal>>
ConeClustering::euclidianClustering(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &ground) {

  // Create a PC and channel for the cone colour
  sensor_msgs::PointCloud cluster_msg;
  sensor_msgs::ChannelFloat32 cone_channel;
  cone_channel.name = "cone_type";

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZINormal>);
  tree->setInputCloud(cloud);

  // Define the parameters for Euclidian clustering
  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointXYZ> cluster_centroids;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(2);
  ec.setMaxClusterSize(200);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  // Iterate over all cluster indices
  for (const auto &iter : cluster_indices) {
    // Create PC of the current cone cluster
    pcl::PointCloud<pcl::PointXYZINormal> cone;
    for (auto it : iter.indices) {
      cone.points.push_back(cloud->points[it]);
    }
    cone.width = cone.points.size();
    cone.height = 1;
    cone.is_dense = true;
    clusters.push_back(cone);

    // Compute centroid of each cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cone, centroid);
    pcl::PointXYZ centroid_pos;
    centroid_pos.x = centroid[0];
    centroid_pos.y = centroid[1];
    centroid_pos.z = centroid[2];
    cluster_centroids.push_back(centroid_pos);
  }

  // For each ground point find the closest centroid
  // if it is within a certain threshold, add the point to that cluster
  for (pcl::PointXYZINormal point : ground->points) {
    float closest_cluster_dist = INFINITY;
    uint16_t closest_cluster_id = 0;

    for (uint16_t cluster_id = 0; cluster_id < clusters.size(); cluster_id++) {
      // AFilter in a cylindrical volume around each centroid
      float dist = std::hypot(point.x - cluster_centroids[cluster_id].x,
                              point.y - cluster_centroids[cluster_id].y);

      if (dist < closest_cluster_dist) {
        closest_cluster_dist = dist;
        closest_cluster_id = cluster_id;
      }
    }

    // If the closest cluster is close enough, add the point to the cluster
    if (closest_cluster_dist < 0.3) {
      clusters[closest_cluster_id].points.push_back(point);
    }
  }

  return clusters;
}

/**
 * @brief Clusters the cones in the final filtered point cloud and returns a
 * vector containing the different clusters
 *
 * This time using String Clustering.
 * @refer Broström, Carpenfelt
 *
 *
 */
std::vector<pcl::PointCloud<pcl::PointXYZINormal>>
ConeClustering::stringClustering(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &ground) {

  // construct clusters for further use
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>> clusters;
  std::vector<pcl::PointXYZINormal>
      cluster_rightmost; // The rightmost point in each cluster
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>> finished_clusters;

  // Iterate over all points, up and down, left to right (elevation & azimuth)
  for (uint16_t i = 0; i < cloud->points.size(); i++) {
    pcl::PointXYZINormal point = cloud->points[i];

    std::vector<uint16_t>
        clusters_to_keep; // These cluster ids will be kept at the end of this
                          // clustering because they can still contain a cone
                          // or they can help exclude other points from being a
                          // cone

    bool found_cluster = false;

    // Iterate over the rightmost point in each cluster
    for (uint16_t cluster_id = 0; cluster_id < cluster_rightmost.size();
         cluster_id++) {
      pcl::PointXYZINormal rightmost = cluster_rightmost[cluster_id];

      // This distance should be calculated using max(delta_azi, delta_r)
      float delta_arc = std::abs(std::atan2(point.x, point.y) -
                                 atan2(rightmost.x, rightmost.y));
      float dist = hypot3d(point.x - rightmost.x, point.y - rightmost.y, 0);

      // A cone is max 285mm wide, check whether this point is within that
      // distance from the rightmost point in each cluster
      if (dist < WIDTH_BIG_CONE * min_distance_factor_) {
        found_cluster = true;

        // Add the point to the cluster
        clusters[cluster_id].push_back(point);

        // Check whether this point is the rightmost point in the cluster
        if (point.y > cluster_rightmost[cluster_id].y) {
          cluster_rightmost[cluster_id] = point;
        }
        clusters_to_keep.push_back(cluster_id);

      }
      // filter other clusters
      else {
        // cluster to far to the left to be considered when adding new points
        if (delta_arc >  max_arc_cluster_) {
          finished_clusters.push_back(clusters[cluster_id]);

          // cluster still needs to be considered
        } else {
          clusters_to_keep.push_back(cluster_id);
        }
      }
    }
    // Remove clusters that cannot represent a cone
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>> new_clusters;
    std::vector<pcl::PointXYZINormal> new_cluster_rightmost;
    std::sort(clusters_to_keep.begin(),
              clusters_to_keep.end()); // Sort the index array
    for (uint16_t id : clusters_to_keep) {
      new_clusters.push_back(clusters[id]);
      new_cluster_rightmost.push_back(cluster_rightmost[id]);
    }

    clusters = new_clusters;
    cluster_rightmost = new_cluster_rightmost;

    // We could not find a cluster where we can add the point to, so create a
    // new one
    if (!found_cluster) {
      pcl::PointCloud<pcl::PointXYZINormal> pc;
      pc.points.push_back(point);
      clusters.push_back(pc);
      cluster_rightmost.push_back(point);
    }
  }

  // add the cluster that where put aside because they were located to far to
  // the left
  for (pcl::PointCloud<pcl::PointXYZINormal> cluster : finished_clusters) {
    clusters.push_back(cluster);
  }

  // determine centroid of each cluster to reduced
  // duplicate calculations
  std::vector<pcl::PointXYZ> cluster_centroids;
  for (pcl::PointCloud<pcl::PointXYZINormal> cluster : clusters) {
    // Compute centroid of each cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cluster, centroid);
    pcl::PointXYZ centroid_pos;
    centroid_pos.x = centroid[0];
    centroid_pos.y = centroid[1];
    centroid_pos.z = centroid[2];
    cluster_centroids.push_back(centroid_pos);
  }

  // For each ground point find the closest centroid
  // if it is within a certain threshold, add the point to that cluster
  for (pcl::PointXYZINormal point : ground->points) {
    float closest_cluster_dist = INFINITY;
    uint16_t closest_cluster_id = 0;

    for (uint16_t cluster_id = 0; cluster_id < clusters.size(); cluster_id++) {
      // AFilter in a cylindrical volume around each centroid
      float dist = std::hypot(point.x - cluster_centroids[cluster_id].x,
                              point.y - cluster_centroids[cluster_id].y);

      if (dist < closest_cluster_dist) {
        closest_cluster_dist = dist;
        closest_cluster_id = cluster_id;
      }
    }

    // If the closest cluster is close enough, add the point to the cluster
    if (closest_cluster_dist < cone_reconstruction_treshold_) {
      clusters[closest_cluster_id].points.push_back(point);
    }
  }

  return clusters;
}

/**
 * @brief Construct a ROS message from the clusters.
 * This message is a pointcloud where each point is colored based on the cluster
 * it belongs to.
 * @returns a sensor_msgs::PointCloud the colored points.
 */
sensor_msgs::PointCloud2 ConeClustering::clustersColoredMessage(
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>> clusters) {

  pcl::PointCloud<pcl::PointXYZRGB> points;
  for (pcl::PointCloud<pcl::PointXYZINormal> cluster : clusters) {
    // choose color of cluster based on position of the first point in the
    // cluster to keep the cluster color as constant as possible
    pcl::PointXYZINormal first = cluster.points[0];
    int color = int((first.x / 0.3) * (first.y / 0.3)) % 21;
    int r = ConeClustering::colors_clusters[color][0];
    int g = ConeClustering::colors_clusters[color][1];
    int b = ConeClustering::colors_clusters[color][2];

    // color the points of the cluster
    for (pcl::PointXYZINormal point : cluster) {
      pcl::PointXYZRGB new_point;
      new_point.x = point.x;
      new_point.y = point.y;
      new_point.z = point.z;
      new_point.r = r;
      new_point.g = g;
      new_point.b = b;
      points.push_back(new_point);
    }
  }

  sensor_msgs::PointCloud2 clusters_colored_msg;
  pcl::toROSMsg(points, clusters_colored_msg);
  return clusters_colored_msg;
}

/**
 * @brief Construct a ROS message from the clusters, containing the position of
 * each cone, as wel as its dimensions.
 *
 * @returns a sensor_msgs::PointCloud containing the information about all the
 * cones in the current frame.
 */
sensor_msgs::PointCloud ConeClustering::constructMessage(
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>> clusters) {
  // Create a PC and channel for: the cone colour, the (x,y,z) dimensions of the
  // cluster and the cone metric
  sensor_msgs::PointCloud cluster_msg;
  sensor_msgs::ChannelFloat32 cone_channel;
  sensor_msgs::ChannelFloat32 x_size_channel;
  sensor_msgs::ChannelFloat32 y_size_channel;
  sensor_msgs::ChannelFloat32 z_size_channel;
  sensor_msgs::ChannelFloat32 cone_metric_channel;

  // name channels
  cone_channel.name = "cone_type";
  x_size_channel.name = "x_width";
  y_size_channel.name = "y_width";
  z_size_channel.name = "z_width";
  cone_metric_channel.name = "cone_metric";

  // iterate over each cluster
  for (pcl::PointCloud<pcl::PointXYZINormal> cluster : clusters) {
    ConeCheck cone_check = coneClassification_.classifyCone(cluster);

    // only add clusters that are likely to be cones
    if (cone_check.is_cone) {
      cluster_msg.points.push_back(cone_check.pos);
      cone_channel.values.push_back(cone_check.color);
      x_size_channel.values.push_back(cone_check.bounds[0]);
      y_size_channel.values.push_back(cone_check.bounds[1]);
      z_size_channel.values.push_back(cone_check.bounds[2]);
      cone_metric_channel.values.push_back(cone_check.cone_metric);
    }
  }

  // add the channels to the message
  cluster_msg.channels.push_back(cone_channel);
  cluster_msg.channels.push_back(x_size_channel);
  cluster_msg.channels.push_back(y_size_channel);
  cluster_msg.channels.push_back(z_size_channel);
  cluster_msg.channels.push_back(cone_metric_channel);
  return cluster_msg;
}

} // namespace ns_lidar