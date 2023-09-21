#include <center_points.hpp>

namespace pathplanning {

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>>
get_center_points(const std::vector<std::vector<double>>& position_cones,
                  const std::vector<int>& classes,
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
    // Perform delaunay triangulation
    Delaunator d(positions);
    
    const std::vector<std::size_t>& triangles = d.triangles;
    const std::vector<double>& coords = d.coords;

    std::vector<std::vector<double>> center_points;
    std::vector<int> center_point_classes;
    std::vector<std::vector<double>> bad_points;

    std::vector<size_t> indices;
    for (size_t i = 0; i < position_cones.size(); ++i)
    {
        indices.push_back(i);
    }

    std::vector<double> filtered_coords;
    std::vector<double> distances;
    std::vector<double> variances;
    std::vector<double> perimeters;

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

        double perimeter = distances[0] + distances[1] + distances[2];

        // Normalize variance by perimeter
        variances.push_back(variance / perimeter);
    }

    double median_variance = calculate_median(variances);

    for (size_t i = 0; i < variances.size(); i++){

        // If below variance threshold, get center points
        if (variances[i] < triangulation_min_var || variances[i] < triangulation_var_threshold * median_variance){
            for (size_t j = 0; j < 3; j++){

                size_t index1 = 2 * triangles[3*i + j];
                size_t index2 = 2 * triangles[(3*i + j + 1) % 3 + 3*i];

                double x_coord = (coords[index1] + coords[index2]) / 2;
                double y_coord = (coords[index1 + 1] + coords[index2 + 1]) / 2;

                // If this is true, this means that the two points that make up the centerpoint
                // are the same color (except for orange cones). So then we have a bad point!

                int compound_class = classes[triangles[3*i + j]] + classes[triangles[(3*i + j + 1) % 3 + 3*i]];

                if (compound_class == 1 || compound_class == 4) {
                    filtered_coords.push_back(coords[2 * triangles[3*i]]);
                    filtered_coords.push_back(coords[2 * triangles[3*i] + 1]);
                    center_points.push_back({x_coord, y_coord});
                } else {
                    bad_points.push_back({x_coord, y_coord});
                }
            }
        } else {
            for (size_t j = 0; j < 3; j++){
                double x_coord = (coords[2 * triangles[3*i + j]] + coords[2 * triangles[(3*i + j + 1) % 3 + 3*i]]) / 2;
                double y_coord = (coords[2 * triangles[3*i + j] + 1] + coords[2 * triangles[(3*i + j + 1) % 3 + 3*i] + 1]) / 2;
                bad_points.push_back({x_coord, y_coord});
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
        ROS_WARN_STREAM("No closest center found!");
    }

    return std::make_tuple(center_points, duplicated_centers, bad_points);
}

} // namespace pathplanning
