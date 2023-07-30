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

#include <triangulator.hpp>




namespace pathplannig {

class Pathplanning {

public:
    Pathplanning(ros::NodeHandle &n);

private:
    ros::NodeHandle n_;

    TransformFrames frametf_ = TransformFrames();

    bool debug_visualisation_;
    std::String vis_namespace_;
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

    ros::Publisher vis_points_ = ros::Publisher();
    ros::Publisher vis_lines_ = ros::Publisher();


    ros::Publisher path_pub_;
    ros::Publisher path_stamped_pub_;

    ros::Subscriber map_sub_;

    Triangulator triangulator_;

    void receive_new_map(const ugr_msgs::ObservationWithCovarianceArrayStamped::ConstPtr& track);
    void compute(const std::vector<std::vector<double>& cones, const std_msgs::Header& header);
};

class TransformFrames
{
public:
    TransformFrames(ros::NodeHandle &n) : nh(n), tfBuffer(), tfListener(tfBuffer)
    {
    }

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

    geometry_msgs::PoseArray pose_transform(const geometry_msgs::PoseArray &pose_array)
    {
        std::string target_frame = nh.param<std::string>("output_frame", "odom");
        geometry_msgs::TransformStamped transform = get_transform(pose_array.header.frame_id, target_frame);

        geometry_msgs::PoseArray pose_array_transformed;
        pose_array_transformed.header.frame_id = target_frame;
        pose_array_transformed.header.stamp = pose_array.header.stamp;

        for (const auto &pose : poseArray.poses)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose = pose;
            pose_stamped.header = pose_array.header;

            geometry_msgs::PoseStamped pose_transformed = tf2::doTransform(pose_stamped, transform);
            pose_array_transformed.poses.push_back(pose_transformed.pose);
        }

        return pose_array_transformed;
    }

    geometry_msgs::PoseStamped get_frame_A_origin_frame_B(const std::string &frame_A, const std::string &frame_B)
    {
        geometry_msgs::PoseStamped origin_A;
        originA.header.frame_id = frame_A;
        originA.header.stamp = ros::Time(0);
        originA.pose.position.x = 0.0;
        originA.pose.position.y = 0.0;
        originA.pose.position.z = 0.0;
        originA.pose.orientation.x = 0.0;
        originA.pose.orientation.y = 0.0;
        originA.pose.orientation.z = 0.0;
        originA.pose.orientation.w = 1.0;

        geometry_msgs::TransformStamped transform = get_transform(frame_A, frame_B);

        geometry_msgs::PoseStamped pose_frame_B = tf2::doTransform(origin_A, transform);
        return pose_frame_B;
    }

private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
};

} // namespace Pathplanning

#endif