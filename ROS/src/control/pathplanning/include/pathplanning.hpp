#ifndef PATHPLANNING_HPP
#define PATHPLANNING_HPP

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <triangulator.hpp>




namespace pathplanning {

class TransformFrames {

public:
    TransformFrames(ros::NodeHandle &n);

    geometry_msgs::TransformStamped get_transform(const std::string &source_frame, const std::string &target_frame)
    {
        try
        {
            return tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.2));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("Cannot find transformation from %s to %s", source_frame.c_str(), target_frame.c_str());
            throw ex;
        }
    }

    nav_msgs::Path pose_transform(const nav_msgs::Path &pose_array)
    {
        std::string target_frame = nh.param<std::string>("output_frame", "odom");
        geometry_msgs::TransformStamped transform = get_transform(pose_array.header.frame_id, target_frame);

        nav_msgs::Path pose_array_transformed;
        pose_array_transformed.header.frame_id = target_frame;
        pose_array_transformed.header.stamp = pose_array.header.stamp;

        for (const auto &pose : pose_array.poses)
        {
            geometry_msgs::PoseStamped pose_transformed;
            tf2::doTransform(pose, pose_transformed, transform);
            pose_array_transformed.poses.push_back(pose_transformed);
        }

        return pose_array_transformed;
    }

    geometry_msgs::PoseStamped get_frame_A_origin_frame_B(const std::string &frame_A, const std::string &frame_B)
    {
        geometry_msgs::PoseStamped origin_A;
        origin_A.header.frame_id = frame_A;
        origin_A.header.stamp = ros::Time(0);
        origin_A.pose.position.x = 0.0;
        origin_A.pose.position.y = 0.0;
        origin_A.pose.position.z = 0.0;
        origin_A.pose.orientation.x = 0.0;
        origin_A.pose.orientation.y = 0.0;
        origin_A.pose.orientation.z = 0.0;
        origin_A.pose.orientation.w = 1.0;

        geometry_msgs::TransformStamped transform = get_transform(frame_A, frame_B);

        geometry_msgs::PoseStamped pose_frame_B;
        tf2::doTransform(origin_A, pose_frame_B, transform);
        return pose_frame_B;
    }

private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
};
class Pathplanning {

public:
    Pathplanning(ros::NodeHandle &n, bool debug_visualisation, std::string vis_namespace,
                    double vis_lifetime, int max_iter, double max_angle_change, 
                    double safety_dist, double triangulation_min_var,
                    double triangulation_var_threshold, double max_path_distance, 
                    double range_front, double range_behind, double range_sides,
                    ros::Publisher vis_points, ros::Publisher vis_lines,
                    double stage1_rect_width_,
                    int stage1_threshold_bad_points_,
                    int stage1_threshold_center_points_,
                    double stage2_rect_width_,
                    int stage2_threshold_bad_points_,
                    int stage2_threshold_center_points_,
                    int max_depth_,
                    double continuous_dist_
    );

private:
    ros::NodeHandle n_;

    TransformFrames frametf_;

    bool debug_visualisation_;
    std::string vis_namespace_;
    double vis_lifetime_;

    int max_iter_;
    double max_angle_change_;
    double safety_dist_;

    double triangulation_min_var_;
    double triangulation_var_threshold_;
    double max_path_distance_;
    double range_front_;
    double range_behind_;
    double range_sides_;
    
    double stage1_rect_width_;
    double stage1_threshold_bad_points_;
    double stage1_threshold_center_points_;
    double stage2_rect_width_;
    double stage2_threshold_bad_points_;
    double stage2_threshold_center_points_;
    double max_depth_;
    double continuous_dist_;

    ros::Publisher vis_points_;
    ros::Publisher vis_lines_;


    ros::Publisher path_pub_;
    ros::Publisher path_stamped_pub_;

    ros::Subscriber map_sub_;

    Triangulator triangulator_;

    void receive_new_map(const ugr_msgs::ObservationWithCovarianceArrayStamped::ConstPtr& track);
    void compute(const std::vector<std::vector<double>>& cones, const std_msgs::Header& header);
};



} // namespace pathplanning

#endif