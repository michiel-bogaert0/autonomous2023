#include "cone_clustering.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

#define VERT_RES_TAN std::tan(0.35 * M_PI / (2.0f * 180))
#define HOR_RES_TAN std::tan(2 * M_PI / (2.0f * 2048))

namespace ns_lidar {
// Constructor
ConeClustering::ConeClustering(ros::NodeHandle &n) : n_(n) {
  // Get parameters
  n.param<std::string>("clustering_method", clustering_method_, "string");
  n.param<double>("cluster_tolerance", cluster_tolerance_, 0.4);
  n.param<double>("point_count_threshold", point_count_threshold_, 0.5);
  n.param<double>("min_distance_factor", min_distance_factor_, 1.5);
  n.param<int>("minimal_points_cone", minimal_points_cone_, 0);
  n.param<float>("minimal_height_cone", minimal_height_cone_, 0.05);
}

/**
 * @brief Clusters the cones in the final filtered point cloud and generates a
 * ROS message.
 *
 * The type of clustering that is applied is chosen by the clustering_method
 * parameter.
 */
sensor_msgs::PointCloud
ConeClustering::cluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr &ground) {
  sensor_msgs::PointCloud cluster_msg;
  if (clustering_method_ == "string") {
    cluster_msg = ConeClustering::stringClustering(cloud);
  } else {
    cluster_msg = ConeClustering::euclidianClustering(cloud, ground);
  }

  return cluster_msg;
}

/**
 * @brief Clusters the cones in the final filtered point cloud and generates a
 * ROS message.
 *
 */
sensor_msgs::PointCloud ConeClustering::euclidianClustering(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &ground) {
  // Create a PC and channel for the cone colour
  sensor_msgs::PointCloud cluster_msg;
  sensor_msgs::ChannelFloat32 cone_channel;
  cone_channel.name = "cone_type";

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);

  // Define the parameters for Euclidian clustering
  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointXYZ> cluster_centroids;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(2);
  ec.setMaxClusterSize(200);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  // Iterate over all cluster indices
  for (const auto &iter : cluster_indices) {
    // Create PC of the current cone cluster
    pcl::PointCloud<pcl::PointXYZI> cone;
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
  for (pcl::PointXYZI point : ground->points) {
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

  return ConeClustering::construct_message(clusters);
}

/**
 * @brief Clusters the cones in the final filtered point cloud and generates a
 * ROS message.
 *
 * This time using String Clustering.
 * @refer Broström, Carpenfelt
 *
 *
 */
sensor_msgs::PointCloud ConeClustering::stringClustering(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {

  // sort point from left to right (because they are ordered from left to right)
  std::sort(cloud->begin(), cloud->end(), leftrightsort);

  std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters;
  std::vector<pcl::PointXYZI> cluster_rightmost; // The rightmost point in each cluster
  std::vector<pcl::PointCloud<pcl::PointXYZI>> finished_clusters;

  // Iterate over all points, up and down, left to right (elevation & azimuth)
  for (uint16_t i = 0; i < cloud->points.size(); i++) {
    pcl::PointXYZI point = cloud->points[i];

    std::vector<uint16_t>
        clusters_to_keep; // These cluster ids will be kept at the end of this
                          // clustering because they can still contain a cone
                          // or they can help exclude other points from being a cone

    bool found_cluster = false;

    // Iterate over the rightmost point in each cluster
    for (uint16_t cluster_id = 0; cluster_id < cluster_rightmost.size();
         cluster_id++) {
      pcl::PointXYZI rightmost = cluster_rightmost[cluster_id];

      // This distance should be calculated using max(delta_azi, delta_r)
      float r_point = ConeClustering::hypot3d(point.x, point.y, point.z);
      float r_rightmost =
          ConeClustering::hypot3d(rightmost.x, rightmost.y, rightmost.z);
      float delta_r = std::abs(r_point - r_rightmost);
      float delta_arc = std::abs(std::atan2(point.x, point.y) -
                                 atan2(rightmost.x, rightmost.y)) *
                        r_rightmost;
      float dist = std::max(delta_r, delta_arc);

      // A cone is max 285mm wide, check whether this point is within that
      // distance from the rightmost point in each cluster
      if (dist < 0.285 * min_distance_factor_) {
        found_cluster = true;

        // Add the point to the cluster
        clusters[cluster_id].push_back(point);

        // Check whether this point is the rightmost point in the cluster
        if (point.y > cluster_rightmost[cluster_id].y) {
          cluster_rightmost[cluster_id] = point;
        }

        // Everytime a cluster is updated, check whether it can still be a cone
        Eigen::Vector4f min;
        Eigen::Vector4f max;
        pcl::getMinMax3D(clusters[cluster_id], min, max);

        float bound_x = std::fabs(max[0] - min[0]);
        float bound_y = std::fabs(max[1] - min[1]);
        float bound_z = std::fabs(max[2] - min[2]);

        // Filter based on the shape of cones
        if (bound_x < 1 && bound_y < 1 && bound_z < 1) {
          // This cluster can still be a cone 
         clusters_to_keep.push_back(cluster_id);
        }
      }
      // filter other clusters
      else {
        // cluster to far to the left to be considered when adding new points
        if (rightmost.y + 0.285 < point.y) {
          finished_clusters.push_back(clusters[cluster_id]);

          // cluster still needs to be considered
        } else {
          clusters_to_keep.push_back(cluster_id);
        }
      }
    }

    // Remove clusters that cannot represent a cone
    std::vector<pcl::PointCloud<pcl::PointXYZI>> new_clusters;
    std::vector<pcl::PointXYZI> new_cluster_rightmost;
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
      pcl::PointCloud<pcl::PointXYZI> pc;
      pc.points.push_back(point);
      clusters.push_back(pc);
      cluster_rightmost.push_back(point);
    }
  }

  // add the cluster that where put aside because they were located to far to
  // the left
  for (pcl::PointCloud<pcl::PointXYZI> cluster : finished_clusters) {
    clusters.push_back(cluster);
  }

  return ConeClustering::construct_message(clusters);
}

/**
 * @brief Construct a ROS message from the clusters, containing the position of
 * each cone, as wel as its dimensions.
 *
 * @returns a sensor_msgs::PointCloud containing the information about all the
 * cones in the current frame.
 */
sensor_msgs::PointCloud ConeClustering::construct_message(
    std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters) {
  // Create a PC and channel for: the cone colour, the (x,y,z) dimensions of the
  // cluster
  sensor_msgs::PointCloud cluster_msg;
  sensor_msgs::ChannelFloat32 cone_channel;
  sensor_msgs::ChannelFloat32 x_size_channel;
  sensor_msgs::ChannelFloat32 y_size_channel;
  sensor_msgs::ChannelFloat32 z_size_channel;

  // name channels
  cone_channel.name = "cone_type";
  x_size_channel.name = "x_width";
  y_size_channel.name = "y_width";
  z_size_channel.name = "z_width";

  // iterate over each cluster
  for (pcl::PointCloud<pcl::PointXYZI> cluster : clusters) {
    ConeCheck cone_check = ConeClustering::isCloudCone(cluster);

    // only add clusters that are likely to be cones
    if (cone_check.is_cone) {
      cluster_msg.points.push_back(cone_check.pos);
      cone_channel.values.push_back(cone_check.color);
      x_size_channel.values.push_back(cone_check.bounds[0]);
      y_size_channel.values.push_back(cone_check.bounds[1]);
      z_size_channel.values.push_back(cone_check.bounds[2]);
    }
  }

  // add the channels to the message
  cluster_msg.channels.push_back(cone_channel);
  cluster_msg.channels.push_back(x_size_channel);
  cluster_msg.channels.push_back(y_size_channel);
  cluster_msg.channels.push_back(z_size_channel);
  return cluster_msg;
}

/**
 * @brief Checks whether a PC represents a cone and which color it has.
 *
 * @returns ConeCheck containing the cones coordinates and whether it is a cone
 */
ConeCheck ConeClustering::isCloudCone(pcl::PointCloud<pcl::PointXYZI> cone) {
  ConeCheck cone_check;

  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  pcl::compute3DCentroid(cone, centroid);
  pcl::getMinMax3D(cone, min, max);

  float bound_x = std::fabs(max[0] - min[0]);
  float bound_y = std::fabs(max[1] - min[1]);
  float bound_z = std::fabs(max[2] - min[2]);


  // filter based on the shape of cones
  if (bound_x < 0.3 && bound_y < 0.3 && bound_z < 0.4 && cone.points.size() >= minimal_points_cone_ && centroid[2] - min[2] > minimal_height_cone_)
  {
    // Calculate the expected number of points that hit the cone
    float dist = ConeClustering::hypot3d(centroid[0], centroid[1], centroid[2]);
    // Using the AMZ formula with w_c = average of x and y widths and r_v=0.35°
    // and r_h=2048 points per rotation
    float num_points = (1 / 2.0f) * (0.325 / (2.0f * dist * VERT_RES_TAN)) *
                       (0.228 / (2.0f * dist * HOR_RES_TAN));

    // We allow for some play in the point count prediction
    if ((std::abs(num_points - cone.points.size()) / num_points) <
        point_count_threshold_) {
      cone_check.pos.x = centroid[0];
      cone_check.pos.y = centroid[1];
      cone_check.pos.z = centroid[2];
      cone_check.bounds[0] = bound_x;
      cone_check.bounds[1] = bound_y;
      cone_check.bounds[2] = bound_z;
      cone_check.is_cone = true;

      // Eigen::Matrix<double, 3, 3> covariance_matrix;
      // pcl::computeCovarianceMatrix(cone, covariance_matrix);

      // ROS_INFO("variance x: %lf", covariance_matrix(1,1));
      // calculate the convexity of the intensity to distinguish between blue
      // and yellow;
      Eigen::MatrixXd X_mat(cone.points.size(), 3);
      Eigen::VectorXd Y_mat(cone.points.size(), 1);

      // setup matrix
      for (int i = 0; i < cone.points.size(); i++) {
        float z_value = cone.points[i].z;
        X_mat(i, 0) = z_value * z_value;
        X_mat(i, 1) = z_value;
        X_mat(i, 2) = 1;

        Y_mat(i) = cone.points[i].intensity;
      }

      // solve ordinary least squares minimisation
      Eigen::VectorXd solution = X_mat.colPivHouseholderQr().solve(Y_mat);

      // determine colour
      if (solution(0) > 0)
        cone_check.color = 1;
      else
        cone_check.color = 0;

      return cone_check;
    }
  }

  cone_check.is_cone = false;
  return cone_check;
}

/**
 * @brief C++17 hypot extension for 3D coordinates
 *
 */
float ConeClustering::hypot3d(float a, float b, float c) {
  return std::sqrt(a * a + b * b + c * c);
}
} // namespace ns_lidar
