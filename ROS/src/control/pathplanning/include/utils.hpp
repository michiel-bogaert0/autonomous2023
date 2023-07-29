#ifndef UTILS_HPP
#define UTILS_HPP

#include <vector>
#include <array>

namespace triangulation {
struct Node {
    double x;
    double y;
    double distance;
    Node* parent; // Pointer to the parent node
    std::vector<Node*> children; // Vector of pointers to child nodes
    double angle;
    double angle_change;

    Node(double x, double y, double distance, Node* parent, const std::vector<Node*>& children, double angle, double angle_change)
        : x(x), y(y), distance(distance), parent(parent), children(children), angle(angle), angle_change(angle_change) {}
};

// Define a utility function to calculate squared distance between 2 points
double distance_squared(double x1, double y1, double x2, double y2);

// Function to check if there is no collision between a node and a point with safety distance around cones
bool no_collision(const Node& parent, const std::array<double, 2>& point, const std::vector<std::array<double, 3>>& cones, double safety_dist_squared);

// Function to get the closest center points to an origin point
std::vector<std::array<double, 2>> get_closest_center(const std::vector<std::array<double, 2>>& center_points, size_t amount, const std::array<double, 2>& origin = {0, 0}, double max_distance = -1);

// Function to sort the center points based on their distance to the origin point
std::vector<std::array<double, 2>> sort_closest_to(const std::vector<std::array<double, 2>>& center_points, const std::array<double, 2>& origin = {0, 0}, double max_distance = -1);
} // namespace triangulation

#endif // UTILS_HPP