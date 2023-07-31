#include "pathplanning.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialise the node
  ros::init(argc, argv, "pathplanning");
  ros::NodeHandle n("~");

  bool debug_visualisation = n.param("debug_visualisation", false);
  std::string vis_namespace = n.param("vis_namespace", std::string("pathplanning_vis"));
  double vis_lifetime = n.param("vis_lifetime", 0.2);

  int max_iter = n.param("max_iter", 100);
  double max_angle_change = n.param("max_angle_change", 0.5);
  double safety_dist = n.param("safety_dist", 1.0);

  double triangulation_min_var = n.param("triangulation_min_var", 100.0);
  double triangulation_var_threshold = n.param("triangulation_var_threshold", 1.2);
  double max_path_distance = n.param("max_path_distance", 6.0);
  double range_front = n.param("range_front", 6.0);
  double range_behind = n.param("range_behind", 0.0);
  double range_sides = n.param("range_sides", 3.0);

  ros::Publisher vis_points = ros::Publisher();
  ros::Publisher vis_lines = ros::Publisher();

  if (debug_visualisation){
    vis_points = n.advertise<visualization_msgs::MarkerArray>("/output/debug/markers", 10);
    vis_lines = n.advertise<geometry_msgs::PoseArray>("/output/debug/poses", 10); 
  }
  // Create a LIDAR class object
  pathplanning::Pathplanning pathplanning(n, debug_visualisation, vis_namespace,
                                          vis_lifetime, max_iter, max_angle_change,
                                          safety_dist, triangulation_min_var,
                                          triangulation_var_threshold, max_path_distance,
                                          range_front, range_behind, range_sides, 
                                          vis_points, vis_lines
  );

  // Spin the node
  while (ros::ok()) {
    // Keep the node alive
    ros::spinOnce();
  }

  return 0;
}
