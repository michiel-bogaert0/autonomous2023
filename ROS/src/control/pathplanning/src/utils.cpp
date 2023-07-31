#include <utils.hpp>

namespace pathplanning {

double distance_squared(double x1, double y1, double x2, double y2)
{
    return std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);
}

bool no_collision(const Node& parent, const std::vector<double>& point, const std::vector<std::vector<double>>& cones, double safety_dist_squared)
{
    double x1 = parent.x;
    double y1 = parent.y;
    double x2 = point[0];
    double y2 = point[1];

    for (const auto& cone : cones)
    {
        double xc = cone[0];
        double yc = cone[1];

        double t_numerator = (x1 - xc) * (x1 - x2) + (y1 - yc) * (y1 - y2);
        double t_denominator = std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);

        double t = t_numerator / t_denominator;
        t = std::max(0.0, std::min(1.0, t));

        double xp = x1 + (x2 - x1) * t;
        double yp = y1 + (y2 - y1) * t;

        double dist_squared = distance_squared(xc, yc, xp, yp);
        if (dist_squared < safety_dist_squared)
        {
            return false;
        }
    }

    return true;
}

std::vector<std::vector<double>> get_closest_center(const std::vector<std::vector<double>>& center_points, size_t amount, const std::vector<double>& origin, double max_distance)
{
    std::vector<double> distances_squared;
    distances_squared.reserve(center_points.size());

    for (const auto& center_point : center_points)
    {
        double dist_squared = distance_squared(origin[0], origin[1], center_point[0], center_point[1]);
        distances_squared.push_back(dist_squared);
    }

    std::vector<std::vector<double>> points_within_distance;
    for (size_t i = 0; i < center_points.size(); i++)
    {
        if (distances_squared[i] < max_distance * max_distance || max_distance == -1)
        {
            points_within_distance.push_back(center_points[i]);
        }
    }

    // If there are not enough points
    if (points_within_distance.size() <= amount)
    {
        return points_within_distance;
    }

    // Get the 'amount' closest center points
    std::vector<size_t> ind(center_points.size());
    std::iota(ind.begin(), ind.end(), 0);

    std::partial_sort(ind.begin(), ind.begin() + amount, ind.end(), [&distances_squared](size_t i1, size_t i2) {
        return distances_squared[i1] < distances_squared[i2];
    });

    std::vector<std::vector<double>> closest_center_points;
    closest_center_points.reserve(amount);
    for (size_t i = 0; i < amount; i++)
    {
        closest_center_points.push_back(center_points[ind[i]]);
    }

    return closest_center_points;
}

std::vector<std::vector<double>> sort_closest_to(const std::vector<std::vector<double>>& center_points, const std::vector<double>& origin, double max_distance)
{
    std::vector<double> distances_squared;
    distances_squared.reserve(center_points.size());

    for (const auto& center_point : center_points)
    {
        double dist_squared = distance_squared(origin[0], origin[1], center_point[0], center_point[1]);
        distances_squared.push_back(dist_squared);
    }

    std::vector<std::vector<double>> points_within_distance;
    std::vector<double> distances_squared_points_within_distance;
    for (size_t i = 0; i < center_points.size(); i++)
    {
        if (distances_squared[i] < max_distance * max_distance || max_distance == -1)
        {
            points_within_distance.push_back(center_points[i]);
            distances_squared_points_within_distance.push_back(distances_squared[i]);
        }
    }

    std::vector<size_t> ind(points_within_distance.size());
    std::iota(ind.begin(), ind.end(), 0);

    std::sort(ind.begin(), ind.end(), [&distances_squared_points_within_distance](size_t i1, size_t i2) {
        return distances_squared_points_within_distance[i1] < distances_squared_points_within_distance[i2];
    });

    std::vector<std::vector<double>> sorted_points_within_distance;
    sorted_points_within_distance.reserve(points_within_distance.size());
    for (size_t i = 0; i < points_within_distance.size(); i++)
    {
        sorted_points_within_distance.push_back(points_within_distance[ind[i]]);
    }

    return sorted_points_within_distance;
}

double calculate_variance(const std::vector<double>& data) {
    // Using Boost Accumulators to calculate variance
    namespace ba = boost::accumulators;
    ba::accumulator_set<double, ba::stats<ba::tag::variance>> acc;

    for (double value : data) {
        acc(value);
    }

    return ba::variance(acc);
}

double calculate_median(const std::vector<double>& data) {
    // Using Boost Accumulators to calculate median
    namespace ba = boost::accumulators;
    ba::accumulator_set<double, ba::stats<ba::tag::median>> acc;

    for (double value : data) {
        acc(value);
    }

    return ba::median(acc);
}

} // namespace pathplanning
