#include<center_points.hpp>

namespace triangulation {

std::tuple<std::vector<std::array<double, 2>>, std::vector<std::array<double, 2>>>
get_center_points(const std::vector<std::array<double, 2>>& position_cones,
                  double triangulation_min_var,
                  double triangulation_var_threshold,
                  double range_front)
{   
    // Need 1D array for Delaunay
    std::vector<double> positions;
    positions.reserve(position_cones.size() * 2);
    for (size_t i = 0; i < position_cones.size(), i++) {
        positions.push_back(position_cones[i][0]);
        positions.push_back(position_cones[i][1]);
    }
    delaunator::Delaunator d(positions);
    
    const std::vector<std::size_t>& triangles = d.triangles;
    const std::vector<double>& coords = delaunator.coords;


    // Perform Delaunay triangulation
    std::vector<std::array<double, 2>> center_points;
    std::vector<std::array<double, 2>> flattened_center_points;
    std::unordered_set<std::array<double, 2>, utils::ArrayHash> unique_points;

    std::vector<size_t> indices;
    for (size_t i = 0; i < position_cones.size(); ++i)
    {
        indices.push_back(i);
    }

    std::vector<std::size_t>& filtered_triangles;
    std::vector<double>& filtered_coords;
    std::vector<std::array<double, 2>> center_points;


    
    for (size_t i = 0; i < triangles.size(); i += 3){
        
        // Variance of lengths within a triangle should be small
        std::array<double, 3> distances;

        for (size_t j = 0; j < 3; j++)
        {
            distances[j] = std::pow(coords[2 * triangles[i + j]] - coords[2 * triangles[(i + j + 1) % 3 + i]], 2) + std::pow(coords[2 * triangles[i + j] + 1] - coords[2 * triangles[(i + j + 1) % 3 + i] + 1], 2);
        }
        double variance = std::variance(distances.begin(), distances.end());

        // If below var, get center points
        if (variance < triangulation_min_var || variance < triangulation_var_threshold * std::median(distances.begin(), distances.end())){
            for (size_t j = 0; j < 3; j++){
                filtered.triangles.push_back(triangles[i]);
                filtered_coords.push_back(coords[2 * triangles[i]]);
                filtered_coords.push_back(coords[2 * triangles[i] + 1]);
                double x_coord = (coords[2 * triangles[i + j]] - coords[2 * triangles[(i + j + 1) % 3 + i]]) / 2;
                double y_coord = (coords[2 * triangles[i + j] + 1] - coords[2 * triangles[(i + j + 1) % 3 + i] + 1]) / 2;
                center_points.push_back({x_coord, y_coord});
            }
        }
    }

    /* Each center point inside of the racing track should have been added as part of two neighbouring triangles
        These duplicated centers should form the raceline (with some odd exceptions that can be filtered out using the tree filtering strategy) 
    */

    // Store the counts of each center point using a std::map
    std::unordered_map<std::array<double, 2>, int> center_point_counts;
    for (const auto& point : center_points) {
        center_point_counts[point]++;
    }

    // Find duplicated centers by iterating through the std::map
    std::vector<std::array<double, 2>> duplicated_centers;
    for (const auto& cnt : center_point_counts) {
        if (cnt.second > 1) {
            duplicated_centers.push_back({cnt.first[0], cnt.first[1]});
        }
    }

    // Add closest center in front of you as this one will not be duplicated
    std::vector<std::array<double, 2>> closest_centers = sort_closest_to(duplicated_centers, [0,0], range_front);
    duplicated_centers.push_back(closest_centers.front());

    return std::make_tuple(center_points, duplicated_centers);
}

std::vector<std::array<double, 2>> filter_center_points(const std::vector<std::array<double, 2>>& center_points,
                                                       const std::vector<std::array<double, 2>>& triangulation_centers,
                                                       const std::vector<std::array<double, 3>>& cones)
{
    std::vector<std::array<double, 2>> filtered_points;

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
            if (closestCones.size() > 3) {
                closestCones.pop();
            }
        }
        
        // Copy to colours array to perform count
        std::vector<int> colours;
        while (!closest_cones.empty()) {
            temp_int_values.push_back(closest_cones.top().second);
            closest_cones.pop();
        }

        if (std::count(colours.begin(), colours.end(), colours[0]) != colours.size())
        {
            filtered_points.push_back(center_point);
        }
    }

    return filtered_points;
}

} // namespace triangulation
