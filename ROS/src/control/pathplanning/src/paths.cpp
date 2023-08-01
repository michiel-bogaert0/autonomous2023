#include <paths.hpp>
#include <ros/ros.h>

namespace pathplanning {
std::pair<Node*, std::vector<Node*>> TriangulationPaths::get_all_paths(
    const std::vector<std::vector<double>>& bad_points,
    const std::vector<std::vector<double>>& center_points,
    const std::vector<std::vector<double>>& cones,
    double range_front
) {
    Node* root_node = new Node(0, 0, 0, nullptr, std::vector<Node*>(), 0, 0);

    // Queue contains node to search, path already searched (corresponding to that node), current depth 
    std::vector<std::tuple<Node*, std::vector<std::array<double, 2>>, int>> queue({std::make_tuple(root_node, std::vector<std::array<double, 2>>({std::array<double,2>({0.0,0.0})}), 0)});

    std::vector<Node*> leaves;
    int iteration = 0;

    while (!queue.empty() && iteration < max_iter) {
        iteration++;

        if (iteration >= this->max_iter)
            break;

        bool found_child = false;

        // Get the next element from the queue
        auto t = queue[0];
        auto parent = std::get<0>(t);
        auto path = std::get<1>(t);
        auto depth = std::get<2>(t);

        queue.erase(queue.begin());

        // First stage is adding as much continuous nodes as possible to reduce search space by alot
        bool child_found = true;

        while (child_found) {
            // Get the closest center points to this element that are within close distance
            std::vector<std::vector<double>> next_nodes;
            next_nodes = sort_closest_to(center_points, {parent->x, parent->y}, this->continuous_dist_);

            child_found = false;

            for (auto next_pos : next_nodes) {
                if (pathplanning::check_if_feasible_child(
                    *parent, 
                    path, 
                    next_pos,
                    bad_points,
                    center_points,
                    cones,
                    this->max_angle_change,
                    this->safety_dist_squared,
                    this->stage1_rect_width_,
                    this->stage1_threshold_bad_points_,
                    this->stage1_threshold_center_points_
                )) {

                    double distance_node = pow(parent->x - next_pos[0], 2) + pow(parent->y - next_pos[1], 2);
                    double angle_node = atan2(
                        next_pos[1] - parent->y, next_pos[0] - parent->x
                    );
                    double angle_change = angle_node - parent->angle;

                    Node* node = new Node(next_pos[0], next_pos[1], distance_node, parent, std::vector<Node*>(), angle_node, angle_change);

                    path.push_back({node->x, node->y});
                    parent->children.push_back(node);
                    parent = node;
                    
                    child_found = true;

                    // One child is enough!
                    break;
                }
            }
        }
        // Now we are at the second stage. We basically look for a node in the distance (to bridge a gap) and add it to the stack to explore further
        // We also limit the depth of this stage
        if (depth >= this->max_depth_) {
            leaves.push_back(parent);
            continue;
        }

        // Get the closest center points to this element that are within range_front
        std::vector<std::vector<double>> next_nodes;
        next_nodes = sort_closest_to(center_points, {parent->x, parent->y}, range_front);
        child_found = false;

        for (auto next_pos : next_nodes) {
            if (pathplanning::check_if_feasible_child(
                    *parent, 
                    path, 
                    next_pos,
                    bad_points,
                    center_points,
                    cones,
                    this->max_angle_change,
                    this->safety_dist_squared,
                    this->stage2_rect_width_,
                    this->stage2_threshold_bad_points_,
                    this->stage2_threshold_center_points_
                )) {

                    double distance_node = pow(parent->x - next_pos[0], 2) + pow(parent->y - next_pos[1], 2);
                    double angle_node = atan2(
                        next_pos[1] - parent->y, next_pos[0] - parent->x
                    );
                    double angle_change = angle_node - parent->angle;

                    Node* node = new Node(next_pos[0], next_pos[1], distance_node, parent, std::vector<Node*>(), angle_node, angle_change);
                    
                    std::vector<std::array<double,2>> new_path(path);
                    new_path.push_back({node->x, node->y});
                    parent->children.push_back(node);

                    queue.push_back(std::make_tuple(node, new_path, depth + 1));
            }
        }

        if (!child_found) {
            leaves.push_back(parent);
        }
    }

    return std::make_pair(root_node, leaves);
}

std::pair<double, double> TriangulationPaths::get_cost_branch(const std::vector<Node*>& branch, const std::vector<std::vector<double>>& cones, double range_front) {
    // angle should not change much over one path
    std::vector<double> angle_changes;
    for (const Node* point : branch) {
        angle_changes.push_back(std::abs(point->angle_change));
    }
    
    double angle_cost = calculate_variance(angle_changes);

    // longer paths usually work better as they make use of more center points
    // having many segments along a path is usually a good path
    std::vector<double> node_distances;
    for (const Node* point : branch) {
        node_distances.push_back(point->distance);
    }
    double distance = std::accumulate(node_distances.begin(), node_distances.end(), 0.0);
    double length_cost = 1 / distance + 10.0 / node_distances.size();

    // get array of blue cones and array of yellow cones sorted by distance
    std::vector<std::vector<double>> sorted_cones = sort_closest_to(cones, std::vector<double>{0.0, 0.0}, range_front);
    std::vector<std::vector<double>> blue_sorted, yellow_sorted;
    for (const std::vector<double>& cone : sorted_cones) {
        if (cone[2] == 0) {
            blue_sorted.push_back(cone);
        } else if (cone[2] == 1) {
            yellow_sorted.push_back(cone);
        }
    }

    // get center between closest blue cone and closest yellow cone
    double center_x = (blue_sorted[0][0] + yellow_sorted[0][0]) / 2;
    double center_y = (blue_sorted[0][1] + yellow_sorted[0][1]) / 2;
    bool center_point_close = false;
    for (const Node* point : branch) {
        if (std::abs(center_x - point->x) < std::numeric_limits<double>::epsilon()
            && std::abs(center_y - point->y) < std::numeric_limits<double>::epsilon()) {
            center_point_close = true;
            break;
        }
    }

    if (center_point_close) {
        // if any point of the path is close to this center point, reduce cost
        length_cost /= 100;
        angle_cost /= 100;
    }

    return std::make_pair(angle_cost, length_cost);
}


} // namespace pathplanning