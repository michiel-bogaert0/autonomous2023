#include <paths.hpp>


namespace pathplanning {
std::pair<Node*, std::vector<Node*>> TriangulationPaths::get_all_paths(
    const std::vector<std::vector<double>>& triangulation_centers,
    const std::vector<std::vector<double>>& center_points,
    const std::vector<std::vector<double>>& cones,
    double range_front
) {
    Node* root_node = new Node(0, 0, 0, nullptr, std::vector<Node*>(), 0, 0);
    std::vector<Node*> queue = {root_node};
    std::vector<Node*> leaves;
    int iteration = 0;

    while (!queue.empty() && iteration < max_iter) {
        iteration++;
        bool found_child = false;

        // Get the next element from the queue
        Node* parent = queue[0];
        queue.erase(queue.begin());

        // Get the closest center points to this element that are within range_front
        std::vector<std::vector<double>> next_nodes;
        next_nodes = sort_closest_to(center_points, {parent->x, parent->y}, range_front);

        // A list of all the abs_angle_change to nodes seen thus far
        std::vector<double> angles_added;

        // Iterate over each next_node, calculate the metrics, and add the node to the tree
        // we also perform some early pruning based on the angle between nodes
        for (const std::vector<double>& next_pos : next_nodes) {
            // Make sure we're not looping from the parent to itself
            if (std::abs(next_pos[0] - parent->x) < std::numeric_limits<double>::epsilon()
                && std::abs(next_pos[1] - parent->y) < std::numeric_limits<double>::epsilon()) {
                continue;
            }

            double angle_node = std::atan2(next_pos[1] - parent->y, next_pos[0] - parent->x);
            double angle_change = angle_node - parent->angle;
            double abs_angle_change = std::min(std::abs(angle_change), 2 * M_PI - std::abs(angle_change));

            // We want to actually physically branch out, so only add one branch for each direction
            // By starting with the closest points, we hope this is a good point for a given direction
            bool within_range = false;
            for (double added_angle : angles_added) {
                if (std::abs(abs_angle_change - added_angle) < this->angle_sensitivity) {
                    within_range = true;
                    break;
                }
            }

            if (within_range) {
                continue;
            }

            // Check that the angle change is within bounds
            if (abs_angle_change > this->max_angle_change) {
                continue;
            }

            double distance_node = (parent->x - next_pos[0]) * (parent->x - next_pos[0]) + (parent->y - next_pos[1]) * (parent->y - next_pos[1]);

            if (no_collision(*parent, next_pos, cones, this->safety_dist_squared)) {
                Node* node = new Node(next_pos[0], next_pos[1], distance_node, parent, std::vector<Node*>(), angle_node, angle_change);

                angles_added.push_back(abs_angle_change);
                queue.push_back(node);
                parent->children.push_back(node);
                found_child = true;
            }
        }

        // Check whether this node is a leaf node
        if (!found_child) {
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