#include "pathplanning.hpp"


namespace pathplanning{

Pathplanning::Pathplanning(ros::NodeHandle &n)
    : n_(n) {

    // Do we want to output visualizations for diagnostics?
    nh.param("~debug_visualisation", this->debug_visualisation_, false);
    nh.param("~vis_namespace", this->vis_namespace_, std::string("pathplanning_vis"));
    nh.param("~vis_lifetime", this->vis_lifetime_, 0.2);

    nh.param("~max_iter", this->max_iter_, 100);
    nh.param("~max_angle_change", this->max_angle_change_, 0.5);
    nh.param("~safety_dist", this->safety_dist_, 1);

    nh.param("~triangulation_min_var", this->triangulation_min_var_, 100);
    nh.param("~triangulation_var_threshold", this->triangulation_var_threshold_, 1.2);
    nh.param("~max_path_distance", this->max_path_distance_, 6.0);
    nh.param("~range_front", this->range_front_, 6.0);
    nh.param("~range_behind", this->range_behind_, 0);
    nh.param("~range_sides", this->range_sides_, 3);

    if (this->debug_visualisation_){
        this->vis_points_ = nh.advertise<visualization_msgs::MarkerArray>("/output/debug/markers", 10);
        this->vis_lines_ = nh.advertise<geometry_msgs::PoseArray>("/output/debug/poses", 10); 
    }

    this->triangulator_ = Triangulator(this->n_, this->triangulation_min_var_, this->triangulation_var_threshold_,
                                 this->max_iter_, this->max_angle_change_, this->max_path_distance_,
                                 this->safety_dist_, this->range_front_, this->range_behind_,
                                 this->range_sides_, this->vis_points_, this->vis_lines_, this->vis_namespace_,
                                 this->vis_lifetime_);
    

    this->path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/output/path", 10);
    this->path_stamped_pub_ = nh_.advertise<nav_msgs::Path>("/output/path_stamped", 10);
    this->map_sub_ = nh_.subscribe("/input/local_map", 10, &PathPlanning::receive_new_map, this);
}

void Pathplanning::receive_new_map(const ugr_msgs::ObservationWithCovarianceArrayStamped::ConstPtr& track)
{
    std::vector<std::vector<double> cones;
    for (const auto& obs_with_cov : track->observations)
    {
        std::vector<double> cone;
        cone[0] = obs_with_cov.observation.location.x;
        cone[1] = obs_with_cov.observation.location.y;
        cone[2] = obs_with_cov.observation.observation_class;
        cones.push_back(cone);
    }

    compute(cones, track->header);
}

void Pathplanning::compute(const std::vector<std::vector<double>>& cones, const std_msgs::Header& header)
{
    std::vector<Node*> path = this->triangulator_.get_path(cones, header);
    
    if (path.empty())
    {
        ROS_INFO_STREAM("No path found");
        return;
    }

    std::vector<geometry_msgs::Pose> poses;
    for (const auto& node : path)
    {
        geometry_msgs::Pose pose;
        pose.position.x = node->x;
        pose.position.y = node->y;
        pose.position.z = 0.0;

        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;

        poses.push_back(pose);
    }

    geometry_msgs::PoseArray output;
    output.header.frame_id = header.frame_id;
    output.poses = poses;
    output.header.stamp = header.stamp;

    geometry_msgs::PoseArray output_transformed = this->frametf_.pose_transform(output);

    this->path_pub_.publish(output_transformed);

    std::vector<geometry_msgs::PoseStamped> poses_stamped;
    for (const auto& pose : output_transformed.poses)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose = pose;
        pose_stamped.header = output_transformed.header;
        poses_stamped.push_back(pose_stamped);
    }

    nav_msgs::Path stamped_output;
    stamped_output.header = output_transformed.header;
    stamped_output.poses = poses_stamped;

    this->path_stamped_pub_.publish(stamped_output);
}


} // namespace pathplanning