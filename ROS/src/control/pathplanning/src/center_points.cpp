#include <center_points.hpp>

namespace pathplanning {

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>>
get_center_points(const std::vector<std::vector<double>>& position_cones,
                  double triangulation_min_var,
                  double triangulation_var_threshold,
                  double range_front)
{   
    // Need 1D array for Delaunay
    std::vector<double> positions;
    positions.reserve(position_cones.size() * 2);
    for (size_t i = 0; i < position_cones.size(); i++) {
        positions.push_back(position_cones[i][0]);
        positions.push_back(position_cones[i][1]);
    }
    Delaunator d(positions);
    
    const std::vector<std::size_t>& triangles = d.triangles;
    const std::vector<double>& coords = d.coords;

    std::vector<std::vector<double>> center_points;

    std::vector<size_t> indices;
    for (size_t i = 0; i < position_cones.size(); ++i)
    {
        indices.push_back(i);
    }

    std::vector<double> filtered_coords;
    std::vector<double> distances;
    std::vector<double> variances;

    for (size_t i = 0; i < triangles.size(); i += 3){
        
        // Variance of lengths within a triangle should be small
        distances.clear();

        for (size_t j = 0; j < 3; j++)
        {   
            size_t index1 = 2 * triangles[i + j];
            size_t index2 = 2 * triangles[(i + j + 1) % 3 + i];
            double distance = std::pow(coords[index1] - coords[index2], 2) + std::pow(coords[index1 + 1] - coords[index2 + 1], 2);
            distances.push_back(distance);
        }
        double variance = calculate_variance(distances);
        variances.push_back(variance);
    }

    double median_variance = calculate_median(variances);

    for (size_t i = 0; i < variances.size(); i++){
        // If below var, get center points
        if (variances[i] < triangulation_min_var || variances[i] < triangulation_var_threshold * median_variance){
            for (size_t j = 0; j < 3; j++){
                filtered_coords.push_back(coords[2 * triangles[3*i]]);
                filtered_coords.push_back(coords[2 * triangles[3*i] + 1]);
                double x_coord = (coords[2 * triangles[3*i + j]] + coords[2 * triangles[(3*i + j + 1) % 3 + 3*i]]) / 2;
                double y_coord = (coords[2 * triangles[3*i + j] + 1] + coords[2 * triangles[(3*i + j + 1) % 3 + 3*i] + 1]) / 2;
                center_points.push_back({x_coord, y_coord});
            }
        }
    }

    /* Each center point inside of the racing track should have been added as part of two neighbouring triangles
        These duplicated centers should form the raceline (with some odd exceptions that can be filtered out using the tree filtering strategy) 
    */

    // Store the counts of each center point using a std::map
    std::map<std::vector<double>, int> center_point_counts;
    for (const auto& point : center_points) {
        center_point_counts[point]++;
    }

    // Find duplicated centers by iterating through the std::map
    std::vector<std::vector<double>> duplicated_centers;
    for (const auto& cnt : center_point_counts) {
        if (cnt.second > 1) {
            duplicated_centers.push_back({cnt.first[0], cnt.first[1]});
        }
    }

    // Add closest center in front of you as this one will not be duplicated
    std::vector<std::vector<double>> closest_centers = sort_closest_to(duplicated_centers, {0.0,0.0}, range_front);
    if (!closest_centers.empty()) {
        duplicated_centers.push_back(closest_centers.front());
    } else {
        ROS_WARN_STREAM("closest_centers is empty. Handling this case accordingly...");
    }

    return std::make_tuple(center_points, duplicated_centers);
}

std::vector<std::vector<double>> filter_center_points(const std::vector<std::vector<double>>& center_points,
                                                       const std::vector<std::vector<double>>& triangulation_centers,
                                                       const std::vector<std::vector<double>>& cones)
{
    std::vector<std::vector<double>> filtered_points;

    // False positive removal
    // Remove points whose closest three cones have the same color

    // Calculate the distance from each center_point to each cone
    for (const auto& center_point : center_points)
    {   
        // Queue to keep distance and colour of three closest cones
        std::priority_queue<std::pair<double, int>> closest_cones;
        for (const auto& cone : cones)
        {   
            double dist = distance_squared(center_point[0], center_point[1], cone[0], cone[1]);
            closest_cones.push(std::make_pair(dist, cone[2]));

            // If there are more than three cones, remove the farthest one
            if (closest_cones.size() > 3) {
                closest_cones.pop();
            }
        }
        
        // Copy to colours array to perform count
        std::vector<int> colours;
        while (!closest_cones.empty()) {
            colours.push_back(closest_cones.top().second);
            closest_cones.pop();
        }

        if (std::count(colours.begin(), colours.end(), colours[0]) != colours.size())
        {
            filtered_points.push_back(center_point);
        }
    }

    return filtered_points;
}

} // namespace pathplanning
