#include "cone_classification.hpp"
#include <algorithm>

#define VERT_RES_TAN std::tan(0.35 * M_PI / (2.0f * 180))
#define HOR_RES_TAN std::tan(2 * M_PI / (2.0f * 2048))

namespace ns_lidar {

// Constructor
ConeClassification::ConeClassification(ros::NodeHandle &n) : n_(n) {
  // Get parameters
  n.param<double>("point_count_threshold", point_count_threshold_, 0.5);
  n.param<int>("minimal_points_cone", minimal_points_cone_, 0);
  n.param<float>("minimal_height_cone", minimal_height_cone_, 0.05);
  n.param<float>("threshold_height_big_cone", threshold_height_big_cone_, 0.40);
  n.param<double>("cone_shape_factor", cone_shape_factor_, 0.3);
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
      centroid[2] - cone.points[0].normal_z > minimal_height_cone_) {
    float dist = hypot3d(centroid[0], centroid[1], centroid[2]);
    float num_points = 0.0;
    bool is_orange = false;

    // check size cloud for orange cone
    if (bound_z > threshold_height_big_cone_) {
      is_orange = true;
    }
    // We allow for some play in the point count prediction
    // and check whether the pointcloud has a shape similar to a cone
    // add the "coneness" metric to the cone_check struct
    double cone_metric =
        ConeClassification::checkShape(cone, centroid, is_orange);
    if (dist != 0.0 && (cone_metric > cone_shape_factor_)) {
      cone_check.pos.x = centroid[0];
      cone_check.pos.y = centroid[1];
      cone_check.pos.z = centroid[2];
      cone_check.bounds[0] = bound_x;
      cone_check.bounds[1] = bound_y;
      cone_check.bounds[2] = bound_z;
      cone_check.is_cone = true;
      cone_check.cone_metric = cone_metric;

      // if the cone is orange based on its height there is no need to compute
      // the color by the intensity
      if (is_orange) {
        cone_check.color = 2;
        return cone_check;
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
    cone_model.height_cone = 0.50;
    cone_model.half_width_cone = 0.980;
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
 * @brief C++17 hypot extension for 3D coordinates
 *
 */
float hypot3d(float a, float b, float c) {
  return std::sqrt(a * a + b * b + c * c);
}

} // namespace ns_lidar