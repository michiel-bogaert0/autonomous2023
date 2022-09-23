#include "cone_classification.hpp"
#include <algorithm>
#include <cmath>

#define VERT_RES_TAN std::tan(0.35 * M_PI / (2.0f * 180))
#define HOR_RES_TAN std::tan(2 * M_PI / (2.0f * 2048))

namespace ns_lidar {


// Constructor
ConeClassification::ConeClassification(ros::NodeHandle &n) : n_(n) {
  // Get parameters
  n.param<double>("point_count_threshold", point_count_threshold_, 0.5);
  n.param<int>("minimal_points_cone", minimal_points_cone_, 0);
  n.param<float>("minimal_height_cone", minimal_height_cone_, 0.05);
  n.param<double>("cone_shape_factor", cone_shape_factor_, 0.3);
}

/**
 * @brief Checks whether a PC represents a cone and which color it has.
 *
 * @returns ConeCheck containing the cones coordinates and whether it is a cone
 */
ConeCheck ConeClassification::classify_cone(const pcl::PointCloud<pcl::PointXYZINormal> cone) {
  
  //compute centroid and bounds of the given pointcloud.
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
  if (cone.points.size() >= minimal_points_cone_ && centroid[2] - cone.points[0].normal_z > minimal_height_cone_)
  {
    float dist = hypot3d(centroid[0], centroid[1], centroid[2]);;
    float num_points = 0.0;
    bool is_orange = false;

    //check size cloud for yellow/blue cone
    if(bound_x < 0.3 && bound_y < 0.3 && bound_z < 0.4)
    {
      // Calculate the expected number of points that hit the cone
      // Using the AMZ formula with w_c = average of x and y widths and r_v=0.35°
      // and r_h=2048 points per rotation
      num_points = (1 / 2.0f) * (0.325 / (2.0f * dist * VERT_RES_TAN)) *
                        (0.228 / (2.0f * dist * HOR_RES_TAN));
    }

    //check size cloud for orange cone
    else if(bound_x < 0.3 && bound_y < 0.3 && bound_z < 0.55){
      num_points = (1 / 2.0f) * (0.505 / (2.0f * dist * VERT_RES_TAN)) *
                        (0.285 / (2.0f * dist * HOR_RES_TAN));
      is_orange = true;
    }

    // We allow for some play in the point count prediction
    // and check whether the pointcloud has a shape similar to a cone
    if (dist != 0.0 && (std::abs(num_points - cone.points.size()) / num_points)  <
        point_count_threshold_ && ConeClassification::checkShape(cone, centroid, is_orange)) {
      cone_check.pos.x = centroid[0];
      cone_check.pos.y = centroid[1];
      cone_check.pos.z = centroid[2];
      cone_check.bounds[0] = bound_x;
      cone_check.bounds[1] = bound_y;
      cone_check.bounds[2] = bound_z;
      cone_check.is_cone = true;

      //if the cone is orange based on its height there is no need to compute the color by the intensity
      if(is_orange){
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
 * @returns a bool that is true if the shape of the given pointcloud is 
 * similar enough to a cone (according to a threshold determined by a rosparam).
 */
bool ConeClassification::checkShape(pcl::PointCloud<pcl::PointXYZINormal> cone, Eigen::Vector4f centroid, bool orange){
  //compute cone model(center + startinglocation)
  ConeModel cone_model;
  cone_model.floor = cone.points[0].normal_z;

  //adapt the cone model for orange cone
  if(orange){
    cone_model.height_cone = 0.505;
    cone_model.half_width_cone = 0.142;
  }

  //compute the center of the cone
  //this center lies further away from the lidar than the centroid of the pointcloud
  //because the pointcloud only sees one half of the cone
  double angle = std::atan2(centroid[1], centroid[0]);
  double translation = cone_model.half_width_cone/3;
  int sign = (centroid[1] > 0) ? 1 : ((centroid[1] <= 0) ? -1 : 0);
  cone_model.x = centroid[0] + std::cos(angle)*translation;
  cone_model.y = centroid[1] + std::sin(angle)*translation*sign;

  //compute metric that shows how similar the pointcloud is to the cone model
  double sum = 0;
  for (pcl::PointXYZINormal point : cone.points){
    double distance = std::sqrt(std::pow(point.x - cone_model.x,2) + std::pow(point.y - cone_model.y,2));
    double expected_height = cone_model.height_cone*(1 - distance/cone_model.half_width_cone);
    double error = std::min(std::abs(1 - ((point.z - point.normal_z)/expected_height)), 1.0);
    sum += 1 - error;
  }
  double cone_metric = sum/cone.points.size();

  return cone_metric > cone_shape_factor_;

}

/**
 * @brief C++17 hypot extension for 3D coordinates
 *
 */
float hypot3d(float a, float b, float c) {
  return std::sqrt(a * a + b * b + c * c);
}

}