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

    while (!queue.empty()) {
        iteration++;

        if (iteration >= this->max_iter)
            break;

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
                    double angle_change = atan2(
                        next_pos[1] - parent->y, next_pos[0] - parent->x
                    );
                    double angle_node = parent->angle + angle_change;

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
    
    std::vector<double> angle_changes;
    std::vector<double> node_distances;
    for (const Node* point : branch) {
        angle_changes.push_back(std::abs(point->angle_change));
        node_distances.push_back(point->distance);
    }
    
    // angle should not change much over one path
    double angle_cost = calculate_variance(angle_changes);

    // longer paths usually work better as they make use of more center points
    // having many segments along a path is usually a good path
    double distance = std::accumulate(node_distances.begin(), node_distances.end(), 0.0);
    double length_cost = 1 / distance + 10.0 / node_distances.size();

    // Iterate over the path, get all cones in a certain distance to line, and apply a penalty every time a cone is on the wrong side
    for (const auto& point : branch) {
        std::vector<double> line_distances_squared(cones.size());
        std::vector<double> distances_squared(cones.size());
        std::vector<std::vector<double>> close_cones;
        std::vector<double> diff_angles;
        std::vector<int> close_classes;

        for (size_t i = 0; i < cones.size(); ++i) {
            line_distances_squared[i] = std::cos(point->angle) * (cones[i][1] - point->y) - std::sin(point->angle) * (cones[i][0] - point->x);
            distances_squared[i] = distance_squared(point->x, point->y, cones[i][0], cones[i][1]);

            if (line_distances_squared[i] < 5 * 3 && distances_squared[i] < 5 * 3) {
                close_cones.push_back(cones[i]);
                diff_angles.push_back(std::atan2(cones[i][1] - point->y, cones[i][0] - point->x) - point->angle);
                close_classes.push_back(cones[i][2]);
            }
        }

        int penalty_amount = 0;
        for (size_t i = 0; i < close_cones.size(); ++i) {
            if (diff_angles[i] > 0.0 && close_classes[i] == 1) {
                penalty_amount++;
            } else if (diff_angles[i] < 0.0 && close_classes[i] == 0) {
                penalty_amount++;
            }
        }

        length_cost += penalty_amount * 10;
        angle_cost += penalty_amount * 10;
    }

    return std::make_pair(angle_cost, length_cost);
}


} // namespace pathplanning