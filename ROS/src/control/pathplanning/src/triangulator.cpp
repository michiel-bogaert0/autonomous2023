#include <triangulator.hpp>

namespace triangulation {
Triangulator::Triangulator(
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
    float vis_lifetime
)
:   n_(n),
    triangulation_min_var_(triangulation_min_var),
    triangulation_var_threshold_(triangulation_var_threshold),
    max_iter_(max_iter),
    max_angle_change_(max_angle_change),
    max_path_distance_(max_path_distance),
    safety_dist_(safety_dist),
    safety_dist_squared_(safety_dist * safety_dist),
    range_front_(range_front),
    range_behind_(range_behind),
    range_sides_(range_sides),
    vis_points_(vis_points),
    vis_lines_(vis_lines),
    vis_namespace_(vis_namespace),
    vis_lifetime_(vis_lifetime)
{
    vis_ = vis_points_ && vis_lines_;
    TriangulationPaths triangulation_paths(max_iter_, max_angle_change_, max_path_distance_, safety_dist_);
}

std::vector<std::vector<double>>& Triangulator::get_path(const std::vector<std::vector<double>& cones, const std_msgs::Header& header)
{
    std::vector<std::vector<double>> filtered_cones;
    std::vector<std::vector<double>> position_cones;
    for (const auto& cone : cones) {
        if (cone[2] <= 1) { // Only keep blue and yellow cones
            position_cones.push_back({cone[0], cone[1]});

            // Only keep cones within a rectangle around the car
            if (cone[0] >= range_behind_ && std::abs(cone[1]) <= range_sides_ && cone[0] <= range_front_) {
                filtered_cones.push_back({cone[0], cone[1]});
            }   
            
        }
    }

    int tries = -1;
    while (filtered_cones.size() < 4) {
        tries++;
        if (tries >= 3){
            return NULL;
        }
        ROS_INFO_STREAM("Not enough cones for triangulation. Trying again with a larger rectangle");

        range_behind_ *= 2;
        range_sides_ *= 2;
        range_front_ *= 2;

        filtered_cones.clear();
        for (const auto& cone : position_cones) {
            if (cone[0] >= range_behind_ && std::abs(cone[1]) <= range_sides_ && cone[0] <= range_front_) {
                filtered_cones.push_back(cone);
            }
        }
    }

    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>> result_center_points;

    std::vector<std::vector<double>> triangulation_centers;
    std::vector<std::vector<double>> center_points;
    
    // Perform triangulation and get the (useful) center points
    result_center_points = get_center_points(filtered_cones, this->triangulation_min_var_, this->triangulation_var_threshold_, this->range_front_);
    triangulation_centers = std::get<0>(result_center_points);
    center_points = std::get<1>(result_center_points);

    // Try to get rid of false positives
    center_points = filter_center_points(center_points, triangulation_centers, cones);

    // TODO: Publish visualisation topics if needed
    
    Node* root_node;
    std::vector<Node*> leaves;
    std::tuple<Node*, std::vector<Node*>> all_paths = this->triangulation_paths.get_all_paths(triangulation_centers, center_points, position_cones, range_front_);
    root_node = std::get<0>(all_paths);
    leaves = std::get<1>(all_paths);

    if (leaves.empty()) {
        return std::vector<Node*>();
    }

    return get_best_path(leaves, cones);
}

std::vector<Node*> Triangulator::get_best_path(const std::vector<Node*>& leaves, const std::vector<std::vector<double>>& cones) {

    std::vector<std::array<double, 2>> costs(leaves.size(), {0.0, 0.0});
    std::vector<std::vector<Node*>> paths;
    std::vector<size_t> path_lengths(1, 0);

    // Iterate each leaf
    for (size_t i = 0; i < leaves.size(), i++) {
        // Find the path connecting this leaf to the root and reverse it
        leaf = leaves[i];
        std::vector<Node*> path;
        Node* parent = leaf;
        while (parent->parent != nullptr) {
            path.push_back(parent);
            parent = parent->parent;
        }
        std::reverse(path.begin(), path.end());

        // Calculate the path cost
        std::tuple<double, double> costs_tuple = this->triangulation_paths.get_cost_branch(path, cones, this->range_front_);
        double angle_cost = std::get<0>(costs_tuple);
        double length_cost = std::get<1>(costs_tuple);
        costs[i] = {angle_cost, length_cost};
        paths.push_back(path);
        path_lengths.push_back(path.size());
    }

    // Normalize costs
    double max_angle_cost = std::max_element(costs.begin(), costs.end(),
        [](const auto& a, const auto& b) {
            return a[0] < b[0];
        }
    )->at(0);
    double max_length_cost = std::max_element(costs.begin(), costs.end(),
        [](const auto& a, const auto& b) {
            return a[1] < b[1];
        }
    )->at(1);
    for (auto& cost : costs) {
        cost[0] /= max_angle_cost;
        cost[1] /= max_length_cost;
    }

    // Calculate total cost for each path
    std::vector<double> total_cost;
    for (const auto& cost : costs) {
        total_cost.push_back(cost[0] + cost[1]);
    }

    // Get the index of the least-cost path
    auto idx = std::min_element(total_cost.begin(), total_cost.end());
    size_t index = std::distance(total_cost.begin(), idx);

    return paths[index];
}

void Triangulator::publish_points(
    const std::vector<std::vector<double>>& points,
    const std_msgs::Header& header,
    const std::string& namespace,
    const std::array<double, 4>& color,
    double scale = 0.1
) {
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < points.size(); i++) {
        const auto& point = points[i];
        visualization_msgs::Marker marker;

        marker.header = header;
        marker.ns = namespace;

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.id = i;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = point[0];
        marker.pose.position.y = point[1];

        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3];

        marker.lifetime = ros::Duration(this->vis_lifetime);

        marker_array.markers.push_back(marker);
    }

    // Assuming you already have a publisher named 'vis_points_pub'
    this->vis_points_.publish(marker_array);
}

void Triangulator::publish_line(
    const std::vector<std::vector<double>>& line,
    const std_msgs::Header& header
) {
    std::vector<geometry_msgs::Pose> poses;

    for (size_t i = 0; i < line.size(); i++) {
        const auto& point = line[i];
        geometry_msgs::Pose pose;
        geometry_msgs::Point position;
        geometry_msgs::Quaternion orientation;

        // Fill position from path point
        position.x = point[0];
        position.y = point[1];
        position.z = 0;

        // Fill orientation (to invalidate)
        orientation.x = 0;
        orientation.y = 0;
        orientation.z = 0;
        orientation.w = 0;

        // Fill pose and add to array
        pose.position = position;
        pose.orientation = orientation;
        poses.push_back(pose);
    }

    geometry_msgs::PoseArray output;
    output.header = header;
    output.poses = poses;

    this->vis_lines_.publish(output);
}

} // namespace triangulation