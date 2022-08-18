#include "ground_removal.hpp"

namespace ns_lidar {

// Constructor
GroundRemoval::GroundRemoval(ros::NodeHandle &n) : n_(n) {
  // Get parameters

  // Ground removal algorithm
  n.param<std::string>("ground_removal_method", ground_removal_method_, "bins");
  n.param<int>("num_iter", num_iter_, 3);
  n.param<int>("num_lpr", num_lpr_, 250);
  n.param<double>("th_seeds", th_seeds_, 1.2);
  n.param<double>("th_dist", th_dist_, 0.1);
  n.param<double>("sensor_height", sensor_height_, 0.4);

  n.param<double>("th_floor", th_floor_, 0.5);
  n.param<int>("radial_buckets", radial_buckets_, 10);
  n.param<int>("angular_buckets", angular_buckets_, 10);
}


/**
 * @brief makes a binarey decision between ground and not-ground for every point.
 *
 * The type of ground_removal that is applied is chosen by the ground_removal_method
 * parameter.
 */
void GroundRemoval::groundRemoval(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points,
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points){

      if(ground_removal_method_ == "zermas"){
        return GroundRemoval::groundRemoval_Zermas(cloud_in, notground_points, ground_points);
      }
      else{
        return GroundRemoval::groundRemoval_Bins(cloud_in, notground_points, ground_points);
      }

    }

/**
 * @brief Removes ground points by binning the points and 
 * estimating the ground for each bin by the lowest point.
 *
 * @refer: based on the paper: "Robust Perception
 * for Formula Student Driverless Racing" by Gustaf Broström and David Carpenfelt 
 * but simplified by not constructin planes as of yet.
 */
void GroundRemoval::groundRemoval_Bins(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points,
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points){

    // compute the number of buckets that will be necesarry
    int bucket_size = angular_buckets_*radial_buckets_;
    pcl::PointCloud<pcl::PointXYZI> buckets[bucket_size];

    // add point to the correct bucket
    for(uint16_t i =0; i< cloud_in->points.size(); i++){
        pcl::PointXYZI point = cloud_in->points[i];
        double hypot = std::hypot(point.x, point.y) - 1;
        double angle = std::atan2(point.x, point.y) - 0.3;
        int angle_bucket = std::floor(angle / (2.5/double(angular_buckets_)));
        int hypot_bucket = std::floor(hypot / (20/double(radial_buckets_)));

        buckets[angular_buckets_*hypot_bucket + angle_bucket].push_back(point);
    }

    // iterate over each bucket
    for(uint16_t i =0 ; i< bucket_size; i++){
        pcl::PointCloud<pcl::PointXYZI> bucket = buckets[i];

        if(bucket.size() != 0){

          //sort bucket from left to right
            std::sort(bucket.begin(), bucket.end(), zsort);

            // decide floor level based on the lowest point in the bucket;
            double floor = bucket.points[0].z;

            // iterate over each point in bucket in decided whether it is part of the ground
            // based on its distance from the floor level
            for(uint16_t p =0; p< bucket.points.size(); p++){
                pcl::PointXYZI point = bucket.points[p];
                if(point.z - floor < th_floor_){
                    ground_points->push_back(point);
                }
                else{
                    point.z -= floor;
                    notground_points->push_back(point);
                }
            }
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
void GroundRemoval::groundRemoval_Zermas(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points,
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points) {

      
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
        pcl::PointXYZI point;
        point.x = cloud_in->points[k].x;
        point.y = cloud_in->points[k].y;
        point.z = cloud_in->points[k].z;
        point.intensity = cloud_in->points[k].intensity;
        ground_points->points.push_back(point);
      } else {
        pcl::PointXYZI point;
        point.x = cloud_in->points[k].x;
        point.y = cloud_in->points[k].y;
        point.z = cloud_in->points[k].z;
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
      it++;
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
} // namespace ns_lidar
