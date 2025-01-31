#include "cone_classification.hpp"
#include <algorithm>
#include <cmath>

#define VERT_RES_TAN std::tan(0.35 * M_PI / (2.0f * 180))
#define HOR_RES_TAN std::tan(2 * M_PI / (2.0f * 2048))

#define BLUE_CONE 0
#define YELLOW_CONE 1
#define LARGE_ORANGE_CONE 2
#define SMALL_ORANGE_CONE 3
#define WHITE_CONE 4

namespace ns_lidar {

// Constructor
ConeClassification::ConeClassification(ros::NodeHandle &n) : n_(n) {
  // Get parameters
  n.param<double>("point_count_threshold", point_count_threshold_, 0.5);
  n.param<int>("minimal_points_cone", minimal_points_cone_, 0);
  n.param<float>("minimal_height_cone", minimal_height_cone_, 0.05);
  n.param<float>("min_threshold_height_big_cone",
                 min_threshold_height_big_cone_, 0.40);
  n.param<float>("max_threshold_height_big_cone",
                 max_threshold_height_big_cone_, 0.60);
  n.param<double>("cone_shape_factor", cone_shape_factor_, 0.3);
  n.param<double>("cone_height_width_factor", height_width_factor_, 0.9);
  n.param<double>("threshold_white_cone", threshold_white_cones_, 10);
  n.param<bool>("use_white_cones", use_white_cones_, false);
  n.param<bool>("use_orange_cones", use_orange_cones_, false);

  n.param<double>("first_tipping_distance", first_tipping_distance_, 10);
  n.param<double>("second_tipping_distance", second_tipping_distance_, 12);
  n.param<double>("zero_value_distance", zero_value_distance_, 21);
  n.param<double>("value_start", value_start_, 1);
  n.param<double>("value_first_tipping_distance", value_first_tipping_distance_,
                  0.9);
  n.param<double>("value_second_tipping_distance",
                  value_second_tipping_distance_, 0.3);
}

/**
 * @brief Checks whether a PC represents a cone and which color it has.
 *
 * @returns ConeCheck containing the cones coordinates and whether it is a cone
 */
ConeCheck ConeClassification::classifyCone(
    const pcl::PointCloud<pcl::PointXYZINormal> cone) {

  // compute centroid and bounds of the given pointcloud.
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  pcl::compute3DCentroid(cone, centroid);
  pcl::getMinMax3D(cone, min, max);

  float bound_x = std::fabs(max[0] - min[0]);
  float bound_y = std::fabs(max[1] - min[1]);
  float bound_z = std::fabs(max[2] - min[2]);

  ConeCheck cone_check;

  // filter based on number of points and height centroid.
  if (cone.points.size() >= minimal_points_cone_ &&
      centroid[2] - cone.points[0].normal_z > minimal_height_cone_ &&
      bound_z > bound_x * height_width_factor_ &&
      bound_z > bound_y * height_width_factor_) {
    float dist = hypot3d(centroid[0], centroid[1], centroid[2]);
    bool is_orange_height = false;

    // check size cloud for orange cone
    if (bound_z > min_threshold_height_big_cone_ &&
        bound_z < max_threshold_height_big_cone_) {

      // If we do not use orange cones, we can throw away this cloud because
      // it is too tall
      if (!use_orange_cones_) {
        cone_check.is_cone = false;
        return cone_check;
      }
      is_orange_height = true;
    }
    // We allow for some play in the point count prediction
    // and check whether the pointcloud has a shape similar to a cone
    // add the "coneness" metric to the cone_check struct
    double cone_metric =
        ConeClassification::checkShape(cone, centroid, is_orange_height);
    if (dist != 0.0 && (cone_metric > cone_shape_factor_)) {
      cone_check.pos.x = centroid[0];
      cone_check.pos.y = centroid[1];
      cone_check.pos.z = centroid[2];
      cone_check.bounds[0] = bound_x;
      cone_check.bounds[1] = bound_y;
      cone_check.bounds[2] = bound_z;
      cone_check.is_cone = true;

      cone_check.cone_belief = ConeClassification::calculateBelief(dist);
      cone_check.color =
          ConeClassification::colorClassification(cone, dist, is_orange_height);
      return cone_check;
    }
  }

  cone_check.is_cone = false;
  return cone_check;
}

/**
 * @brief determines the color class of the cone by calculating the convexity of
 * intensity values
 *
 * @returns a number representing the color of the cone
 *
 * The main idea behind this function is based on:
 * @ref
 * https://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=9069372&fileOId=9069373
 */
float ConeClassification::colorClassification(
    pcl::PointCloud<pcl::PointXYZINormal> cone, float dist,
    bool potential_orange) {
  if (use_white_cones_ && dist > threshold_white_cones_) {
    return WHITE_CONE;
  }
  // if the cone is orange based on its height there is no need to compute
  // the color by the intensity
  if (potential_orange) {
    return LARGE_ORANGE_CONE;
  }

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
    return YELLOW_CONE;
  else
    return BLUE_CONE;
}

/**
 * @brief determines whether the given pointcloud resembles a cone.
 *
 * @returns a double representing the "coneness metric"
 *
 * The main idea behind this function is based on:
 * @ref
 * https://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=9069372&fileOId=9069373
 */
double
ConeClassification::checkShape(pcl::PointCloud<pcl::PointXYZINormal> cone,
                               Eigen::Vector4f centroid, bool orange) {
  // compute cone model(center + startinglocation)
  ConeModel cone_model;
  cone_model.floor = cone.points[0].normal_z;

  // adapt the cone model for orange cone
  if (orange) {
    cone_model.height_cone = 0.505;
    cone_model.half_width_cone = 0.142;
  }

  // compute the center of the cone
  // this center lies further away from the lidar than the centroid of the
  // pointcloud because the pointcloud only sees one half of the cone
  double angle = std::atan2(centroid[1], centroid[0]);
  double translation = cone_model.half_width_cone / 3;
  int sign = (centroid[1] > 0) ? 1 : ((centroid[1] <= 0) ? -1 : 0);
  cone_model.x = centroid[0] + std::cos(angle) * translation;
  cone_model.y = centroid[1] + std::sin(angle) * translation * sign;

  Eigen::MatrixXf cone_matrix = cone.getMatrixXfMap();

  // calculate distance
  cone_matrix.row(0) =
      Eigen::square((cone_matrix.row(0).array() - cone_model.x));
  cone_matrix.row(1) =
      Eigen::square((cone_matrix.row(1).array() - cone_model.y));
  cone_matrix.row(0) = (cone_matrix.row(0) + cone_matrix.row(1)).cwiseSqrt();

  // calculate expected height of point
  cone_matrix.row(0) =
      (1 - (cone_matrix.row(0) / cone_model.half_width_cone).array()) *
      cone_model.height_cone;

  // compute error term
  cone_matrix.row(3) = (cone_matrix.row(2) - cone_matrix.row(6))
                           .cwiseQuotient(cone_matrix.row(0));
  cone_matrix.row(3) = (1 - cone_matrix.row(3).array()).cwiseAbs();
  cone_matrix.row(3) = cone_matrix.row(3).cwiseMin(1);
  cone_matrix.row(3) = 1 - cone_matrix.row(3).array();

  // compute the average
  double cone_metric = cone_matrix.row(3).sum() / cone_matrix.cols();

  return cone_metric;
}

/**
 * @brief determines The belief in the color of the observation
 *
 * @returns a double representing the belief from 0 to 100%
 */
double ConeClassification::calculateBelief(float dist) {

  double slope_1 =
      (value_first_tipping_distance_ - value_start_) / first_tipping_distance_;
  double slope_2 = -value_second_tipping_distance_ /
                   (zero_value_distance_ - second_tipping_distance_);

  if (dist > zero_value_distance_) {
    return 0;
  }
  if (dist < first_tipping_distance_) {
    return value_start_ + slope_1 * dist;
  }
  if (dist > second_tipping_distance_) {
    return value_second_tipping_distance_ +
           slope_2 * (dist - second_tipping_distance_);
  } else {
    return (value_first_tipping_distance_ - value_second_tipping_distance_) /
               (1 +
                exp(12 * (dist - first_tipping_distance_) /
                        (second_tipping_distance_ - first_tipping_distance_) -
                    6)) +
           value_second_tipping_distance_;
  }
}

/**
 * @brief C++17 hypot extension for 3D coordinates
 *
 */
float hypot3d(float a, float b, float c) {
  return std::sqrt(a * a + b * b + c * c);
}

} // namespace ns_lidar