#ifndef TRIANGULATOR_HPP
#define TRIANGULATOR_HPP

#include <ros/ros.h>
#include <center_points.hpp>
#include <paths.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

#include <vector>
#include <array>
#include <tuple>
#include <cmath> 

namespace pathplanning {
class Triangulator {

public:
    Triangulator(
        ros::NodeHandle &n,
        float triangulation_min_var,
        float triangulation_var_threshold,
        int max_iter,
        float max_angle_change,
        float max_path_distance,
        float safety_dist,
        float range_front,
        float range_behind,
        float range_sides,
        ros::Publisher vis_points,
        ros::Publisher vis_lines,
        std::string vis_namespace,
        float vis_lifetime,
        double stage1_rect_width_,
        int stage1_threshold_bad_points_,
        int stage1_threshold_center_points_,
        double stage2_rect_width_,
        int stage2_threshold_bad_points_,
        int stage2_threshold_center_points_,
        int max_depth_,
        double continuous_dist_
    );

    std::vector<Node*> get_path(const std::vector<std::vector<double>>& cones, const std_msgs::Header& header);

private:
    ros::NodeHandle &n_;
    float triangulation_min_var_;
    float triangulation_var_threshold_;
    int max_iter_;
    float max_angle_change_;
    float max_path_distance_;
    float safety_dist_;
    float safety_dist_squared_;
    float range_front_;
    float range_behind_;
    float range_sides_;
    bool vis_;
    ros::Publisher vis_points_;
    ros::Publisher vis_lines_;
    std::string vis_namespace_;
    float vis_lifetime_;

    TriangulationPaths triangulation_paths;

    void publish_points(const std::vector<std::vector<double>>& points, const std_msgs::Header& header, const std::string& nsp, const std::array<double, 4>& color, double scale);
    void publish_line(const std::vector<std::vector<double>>& line, const std_msgs::Header& header);

    std::vector<Node*> get_best_path(const std::vector<Node*>& leaves, const std::vector<std::vector<double>>& cones);
};

} // namespace pathplanning

#endif