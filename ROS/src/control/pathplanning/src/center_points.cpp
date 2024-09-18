#include <center_points.hpp>

namespace pathplanning {

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>,
           std::vector<std::vector<double>>>
get_center_points(const std::vector<std::vector<double>> &position_cones,
                  const std::vector<int> &classes, double triangulation_max_var,
                  double triangulation_var_threshold, double range_front) {
  // Need 1D array for Delaunay
  std::vector<double> positions;
  positions.reserve(position_cones.size() * 2);
  for (size_t i = 0; i < position_cones.size(); i++) {
    positions.push_back(position_cones[i][0]);
    positions.push_back(position_cones[i][1]);
  }
  // Perform delaunay triangulation
  Delaunator d(positions);

  const std::vector<std::size_t> &triangles_indices = d.triangles;
  const std::vector<double> &coords = d.coords;

  std::vector<std::vector<double>> center_points;
  //   std::vector<int> center_point_classes;
  std::vector<std::vector<double>> bad_points;

  std::vector<Triangle> triangles;
  std::vector<double> distances;
  std::vector<double> variances;
  //   std::vector<double> perimeters;

  for (size_t i = 0; i < triangles_indices.size(); i += 3) {

    // Variance of lengths within a triangle should be small
    distances.clear();

    size_t index1 = 2 * triangles_indices[i];
    size_t index2 = 2 * triangles_indices[i+1];
    size_t index3 = 2 * triangles_indices[i+2];

    TrianglePoint point1(coords[index1], coords[index1 + 1], classes[index1]);
    TrianglePoint point2(coords[index2], coords[index2 + 1], classes[index2]);
    TrianglePoint point3(coords[index3], coords[index3 + 1], classes[index3]);
    Triangle triangle(point1, point2, point3);

    distances.push_back(triangle.side1);
    distances.push_back(triangle.side2);
    distances.push_back(triangle.side3);

    double variance = calculate_variance(distances);

    double perimeter = triangle.side1 + triangle.side2 + triangle.side3;

    // Normalize variance by perimeter
    variances.push_back(variance / perimeter);

    triangles.push_back(triangle);
  }

  double median_variance = calculate_median(variances);

  for (size_t i = 0; i < variances.size(); i++) {


    for (size_t j = 0; j < 3; j++) {

      double x_coord = (triangles[i].points[j].x + triangles[i].points[(j+1)%3].x) /2;
      double y_coord = (triangles[i].points[j].y + triangles[i].points[(j+1)%3].y) /2;

      // If below variance threshold, get center points
      if (variances[i] < triangulation_max_var ||
          variances[i] < triangulation_var_threshold * median_variance) {

        // Check if the 2 points that make up the centerpoint are not
        // the same color (except for orange cones). If this is false, we
        // have a bad point!
        int compound_class = triangles[i].points[j].colorIndex + triangles[i].points[(j+1)%3].colorIndex;

        if (compound_class == 1 || triangles[i].points[j].colorIndex == 2 || triangles[i].points[(j+1)%3].colorIndex == 2){
          center_points.push_back({x_coord,y_coord});
        } else {
          bad_points.push_back({x_coord,y_coord});
        }
      }else{
        bad_points.push_back({x_coord,y_coord});
      }
    }
  }

  /* Each center point inside of the racing track should have been added as part
     of two neighbouring triangles These duplicated centers should form the
     raceline (with some odd exceptions that can be filtered out using the tree
     filtering strategy)
  */

  // Store the counts of each center point using a std::map
  std::map<std::vector<double>, int> center_point_counts;
  for (const auto &point : center_points) {
    center_point_counts[point]++;
  }

  // Find duplicated centers by iterating through the std::map
  std::vector<std::vector<double>> duplicated_centers;
  for (const auto &cnt : center_point_counts) {
    if (cnt.second > 1) {
      duplicated_centers.push_back({cnt.first[0], cnt.first[1]});
    }
  }

  // Add closest center in front of you as this one will not be duplicated
  std::vector<std::vector<double>> closest_centers =
      sort_closest_to(center_points, {0.0, 0.0}, range_front);
  if (!closest_centers.empty()) {
    duplicated_centers.push_back(closest_centers.front());
  } else {
    ROS_WARN_STREAM("No closest center found!");
  }

  return std::make_tuple(center_points, duplicated_centers, bad_points);
}

} // namespace pathplanning