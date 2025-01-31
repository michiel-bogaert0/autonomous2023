#include "ground_removal.hpp"

namespace ns_lidar {

// Constructor
GroundRemoval::GroundRemoval(ros::NodeHandle &n) : n_(n) {
  // Get parameters

  // Preprocessing parameters
  n.param<double>("min_distance", min_distance_, 1.0);
  n.param<double>("max_distance", max_distance_, 21.0);
  n.param<double>("min_angle", min_angle_, 0.3);
  n.param<double>("max_angle", max_angle_, 2.8);

  // Ground removal algorithm
  n.param<std::string>("ground_removal_method", ground_removal_method_, "bins");
  n.param<int>("num_iter", num_iter_, 3);
  n.param<int>("num_lpr", num_lpr_, 250);
  n.param<double>("th_seeds", th_seeds_, 1.2);
  n.param<double>("th_dist", th_dist_, 0.1);
  n.param<double>("sensor_height", sensor_height_, 0.4);

  n.param<double>("th_floor", th_floor_, 0.5);
  n.param<double>("small_radial_bucket_length", small_radial_bucket_length_,
                  0.3);
  n.param<double>("big_radial_bucket_length", big_radial_bucket_length_, 4);
  n.param<int>("angular_buckets", angular_buckets_, 10);
  n.param<double>("radial_bucket_tipping_point", radial_bucket_tipping_point_,
                  10);
  n.param<bool>("noisy_environment", noisy_environment_, false);
  n.param<double>("max_bucket_height", max_bucket_height_, 0.35);
  n.param<int>("min_points_per_bucket", min_points_per_bucket_, 8);
  n.param<int>("max_points_per_bucket", max_points_per_bucket_, 200);
  n.param<bool>("use_slope", use_slope_, true);
  n.param<int>("color_factor", factor_color_, 5);
}

/**
 * @brief makes a binarey decision between ground and not-ground for every
 * point.
 *
 * The type of ground_removal that is applied is chosen by the
 * ground_removal_method parameter.
 */
void GroundRemoval::groundRemoval(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground_points) {

  if (ground_removal_method_ == "zermas") {
    return GroundRemoval::groundRemovalZermas(cloud_in, notground_points,
                                              ground_points);
  } else {
    return GroundRemoval::groundRemovalBins(cloud_in, notground_points,
                                            ground_points);
  }
}

/**
 * @brief Removes ground points by binning the points and
 * estimating the ground for each bin by the lowest point.
 *
 * @refer: based on the paper: "Robust Perception
 * for Formula Student Driverless Racing" by Gustaf Broström and David
 * Carpenfelt but simplified by not constructin planes as of yet.
 */
void GroundRemoval::groundRemovalBins(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground_points) {

  // Calculate the number of buckets between 1 meter and the tipping point
  int number_of_small_buckets =
      std::ceil((radial_bucket_tipping_point_ - min_distance_) /
                small_radial_bucket_length_);
  // Calculate the number of big buckets between the tipping point and the max
  // distance point.
  int number_of_big_buckets =
      std::ceil((max_distance_ - radial_bucket_tipping_point_) /
                big_radial_bucket_length_);

  int radial_buckets_ = number_of_small_buckets + number_of_big_buckets;
  // compute the number of buckets that will be necesarry
  int bucket_size = angular_buckets_ * radial_buckets_;
  pcl::PointCloud<pcl::PointXYZI> buckets[bucket_size];

  // add point to the correct bucket
  for (uint16_t i = 0; i < cloud_in->points.size(); i++) {
    pcl::PointXYZI point = cloud_in->points[i];

    // let hypot and angle start from 0 instead of 1 and min_angle respectively
    double hypot = std::hypot(point.x, point.y) - min_distance_;
    double angle = max_angle_ - std::atan2(point.x, point.y);

    // Calculate which bucket the points falls into
    int angle_bucket = std::floor(
        angle / ((max_angle_ - min_angle_) / double(angular_buckets_)));

    // calculate which bucket a specific points belongs to
    int hypot_bucket = 0;
    if (hypot < radial_bucket_tipping_point_ - min_distance_) {
      hypot_bucket = std::floor(hypot / small_radial_bucket_length_);
    } else {
      hypot_bucket =
          std::floor((hypot - (radial_bucket_tipping_point_ - min_distance_)) /
                     big_radial_bucket_length_) +
          number_of_small_buckets;
    }

    // add point to allocated bucket
    buckets[radial_buckets_ * angle_bucket + hypot_bucket].push_back(point);
  }

  pcl::PointXYZ prev_centroid;
  pcl::PointXYZ prev2_centroid;

  // iterate over each bucket
  for (uint16_t i = 0; i < bucket_size; i++) {
    pcl::PointCloud<pcl::PointXYZI> bucket = buckets[i];

    if (bucket.size() != 0) {

      // throw away buckets with not enough or too much points in noisy
      // environment
      if (noisy_environment_ && ((bucket.size() < min_points_per_bucket_) ||
                                 (bucket.size() > max_points_per_bucket_))) {
        continue;
      }

      // sort bucket from bottom to top
      std::sort(bucket.begin(), bucket.end(), zsort);

      // taken the 10% lowest points
      int number_of_points = std::max(int(std::ceil(bucket.size() / 10)), 1);

      // calculate the height difference between the 10% highest and 10% lowest
      // points throw the bin away if this value is too big, only in noisy
      // environment
      if (noisy_environment_ &&
          (bucket.points[std::ceil(0.9 * bucket.size())].z -
               bucket.points[std::ceil(0.1 * bucket.size())].z >
           max_bucket_height_)) {
        continue;
      }
      pcl::PointCloud<pcl::PointXYZI> expected_ground_points;
      for (int j = 0; j < number_of_points; j++) {
        expected_ground_points.push_back(bucket.points[j]);
      }

      // calculate the averge floor level from these points
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(expected_ground_points, centroid);
      pcl::PointXYZ centroid_pos;
      centroid_pos.x = centroid[0];
      centroid_pos.y = centroid[1];
      centroid_pos.z = centroid[2];

      // classify ground points for previous bucket if it exists
      if (i % radial_buckets_ != 0) {
        process_bucket(buckets[i - 1], prev2_centroid, prev_centroid,
                       centroid_pos,
                       (i - 1 % radial_buckets_ == 0) ? PositionEnum::BEGIN
                                                      : PositionEnum::MID,
                       notground_points, ground_points);
      }

      // calculate ground for current bucket if it is the last radial bucket in
      // an angular segment
      if ((i + 1) % radial_buckets_ == 0) {
        process_bucket(bucket, prev_centroid, centroid_pos, centroid_pos,
                       PositionEnum::END, notground_points, ground_points);
      }

      // edge case for when there is only one radial bucket
      if (radial_buckets_ == 1) {
        process_bucket(bucket, prev_centroid, centroid_pos, centroid_pos,
                       PositionEnum::END, notground_points, ground_points);
      }

      // setup variable for next bucket;
      prev2_centroid = prev_centroid;
      prev_centroid = centroid_pos;
    }
  }
}

/**
 * @brief Removes ground points
 *
 * @refer: Based on the code https://github.com/chrise96/3D_Ground_Segmentation,
 *   which in turn is based on a paper by Zermas et al.
 *   "Fast segmentation of 3D point clouds: a paradigm on GroundRemoval data for
 * Autonomous Vehicle Applications"
 *
 */
void GroundRemoval::groundRemovalZermas(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground_points) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points(
      new pcl::PointCloud<pcl::PointXYZI>());

  // 1. Extract initial ground seeds
  extractInitialSeeds(cloud_in, seed_points);

  // 2. Ground plane fit mainloop
  // The points belonging to the ground surface are used as seeds
  // for the refined estimation of a new plane model and the process
  // repeats for num_iter_ number of times
  for (int i = 0; i < num_iter_; ++i) {
    /// 3. Estimate the plane model
    model_t model = estimatePlane(*seed_points);

    ground_points->clear();
    notground_points->clear();

    // Pointcloud to matrix
    Eigen::MatrixXf points_matrix(cloud_in->points.size(), 3);
    size_t j = 0u;
    for (auto p : (*cloud_in).points) {
      points_matrix.row(j++) << p.x, p.y, p.z;
    }

    // Ground plane model
    Eigen::VectorXf result = points_matrix * model.normal_n;

    // Threshold filter: N^T xi + d = dist < th_dist ==> N^T xi < th_dist - d
    double th_dist_d_ = th_dist_ - model.d;

    // 4. Identify which of the points belong to the ground and non ground
    for (int k = 0; k < result.rows(); ++k) {
      if (result[k] < th_dist_d_) {
        // TODO think about a more optimized code for this part
        pcl::PointXYZINormal point;
        point.x = cloud_in->points[k].x;
        point.y = cloud_in->points[k].y;
        point.z = cloud_in->points[k].z;
        point.intensity = cloud_in->points[k].intensity;
        ground_points->points.push_back(point);
      } else {
        pcl::PointXYZINormal point;
        point.x = cloud_in->points[k].x;
        point.y = cloud_in->points[k].y;
        point.z = result[k];
        point.intensity = cloud_in->points[k].intensity;
        notground_points->points.push_back(point);
      }
    }
  }
}

/**
 * @brief Use the set of seed points to estimate the initial plane model
 * of the ground surface.
 */
model_t GroundRemoval::estimatePlane(
    const pcl::PointCloud<pcl::PointXYZI> &seed_points) {
  Eigen::Matrix3f cov_matrix(3, 3);
  Eigen::Vector4f points_mean;
  model_t model;

  // Compute covariance matrix in single pass
  pcl::computeMeanAndCovarianceMatrix(seed_points, cov_matrix, points_mean);

  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(
      cov_matrix, Eigen::DecompositionOptions::ComputeFullU);

  // Use the least singular vector as normal
  model.normal_n = (svd.matrixU().col(2));

  // Mean ground seeds value
  Eigen::Vector3f seeds_mean = points_mean.head<3>();

  // According to normal.T * [x,y,z] = -d
  model.d = -(model.normal_n.transpose() * seeds_mean)(0, 0);

  return model;
}

/**
 * @brief Extract a set of seed points with low height values.
 */
void GroundRemoval::extractInitialSeeds(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points) {
  // Sort on z-axis values (sortOnHeight)
  std::vector<pcl::PointXYZI> cloud_sorted((*cloud_in).points.begin(),
                                           (*cloud_in).points.end());
  sort(cloud_sorted.begin(), cloud_sorted.end(),
       [](pcl::PointXYZI p1, pcl::PointXYZI p2) {
         // Similar to defining a bool function
         return p1.z < p2.z;
       });

  // Negative outlier error point removal.
  // As there might be some error mirror reflection under the ground
  std::vector<pcl::PointXYZI>::iterator it = cloud_sorted.begin();
  for (size_t i = 0; i < cloud_sorted.size(); ++i) {
    // We define the outlier threshold -1.5 times the height of the
    // GroundRemoval sensor
    if (cloud_sorted[i].z < -1.5 * sensor_height_) {
      ++it;
    } else {
      // Points are in incremental order. Therefore, break loop if here
      break;
    }
  }
  // Remove the outlier points
  cloud_sorted.erase(cloud_sorted.begin(), it);

  // Find the Lowest Point Representive (LPR) of the sorted point cloud
  double LPR_height = 0.;
  for (int i = 0; i < num_lpr_; i++) {
    LPR_height += cloud_sorted[i].z;
  }
  LPR_height /= num_lpr_;

  // Iterate, filter for height less than LPR_height + th_seeds_
  (*seed_points).clear();
  for (size_t i = 0; i < cloud_sorted.size(); ++i) {
    if (cloud_sorted[i].z < LPR_height + th_seeds_) {
      (*seed_points).points.push_back(cloud_sorted[i]);
    }
  }
}

/**
 * @brief process one bucket and decide for each point whether it belong to the
 * ground or not
 */
void GroundRemoval::process_bucket(
    pcl::PointCloud<pcl::PointXYZI> bucket, pcl::PointXYZ prev_centroid,
    pcl::PointXYZ current_centroid, pcl::PointXYZ next_centroid,
    PositionEnum position,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr notground_points,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground_points) {

  pcl::PointCloud<pcl::PointXYZINormal> ng_points;
  for (uint16_t p = 0; p < bucket.points.size(); p++) {
    double floor =
        use_slope_ ? calculate_ground(prev_centroid, current_centroid,
                                      next_centroid, bucket.points[p], position)
                   : current_centroid.z;
    pcl::PointXYZINormal point;
    point.x = bucket.points[p].x;
    point.y = bucket.points[p].y;
    point.z = bucket.points[p].z;
    point.intensity = bucket.points[p].intensity;
    point.normal_z = floor;
    if (point.z - floor < th_floor_) {
      ground_points->push_back(point);
    } else {
      ng_points.push_back(point);
    }
  }
  std::sort(ng_points.begin(), ng_points.end(), GroundRemoval::leftrightsort);
  for (pcl::PointXYZINormal point : ng_points) {
    notground_points->push_back(point);
  }
}

/**
 * @brief calculate the ground level for a specific point
 *  position: -1 --> first radial bucket
 *  position: 0 --> not first or last radial bucket
 *  position: +1 --> last radial bucket
 */
double GroundRemoval::calculate_ground(pcl::PointXYZ prev_centroid,
                                       pcl::PointXYZ current_centroid,
                                       pcl::PointXYZ next_centroid,
                                       pcl::PointXYZI point,
                                       PositionEnum position) {
  double r_current = std::hypot(current_centroid.x, current_centroid.y);
  double r_point = std::hypot(point.x, point.y);
  double r_prev = position == PositionEnum::BEGIN
                      ? 0
                      : std::hypot(prev_centroid.x, prev_centroid.y);
  double r_next = position == PositionEnum::END
                      ? 100
                      : std::hypot(next_centroid.x, next_centroid.y);

  double z_prev =
      position == PositionEnum::BEGIN ? current_centroid.z : prev_centroid.z;
  double z_next =
      position == PositionEnum::END ? current_centroid.z : next_centroid.z;

  double floor = 0.0;
  if (r_point <= r_current) {
    floor = z_prev + (r_point - r_prev) / (r_current - r_prev) *
                         (current_centroid.z - z_prev);
  } else {
    floor = current_centroid.z + (r_point - r_current) / (r_next - r_current) *
                                     (z_next - current_centroid.z);
  }
  return floor;
}

/**
 * @brief Colors the pointcloud in from first point to last point
 * with increasing luminance values.
 * This makes it easier to see in which order points arrive at clustering.
 */
sensor_msgs::PointCloud2 GroundRemoval::publishColoredGround(
    pcl::PointCloud<pcl::PointXYZINormal> points,
    const sensor_msgs::PointCloud2 &msg) {
  int i = 0;
  pcl::PointCloud<pcl::PointXYZRGB> new_points;
  // color the points of the cluster
  for (pcl::PointXYZINormal point : points) {
    int intens = int(i++ / factor_color_) % 256;
    pcl::PointXYZRGB new_point;
    new_point.x = point.x;
    new_point.y = point.y;
    new_point.z = point.z;
    new_point.r = intens;
    new_point.g = intens;
    new_point.b = intens;
    new_points.push_back(new_point);
  }

  sensor_msgs::PointCloud2 ground_colored_msg;
  pcl::toROSMsg(new_points, ground_colored_msg);
  ground_colored_msg.header.frame_id = msg.header.frame_id;
  ground_colored_msg.header.stamp = msg.header.stamp;
  return ground_colored_msg;
}

} // namespace ns_lidar
