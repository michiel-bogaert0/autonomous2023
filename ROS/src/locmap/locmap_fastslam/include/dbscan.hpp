// Credits: https://github.com/Eleobert/dbscan

#include <vector>
#include <Eigen/Dense>

auto dbscanVectorXf(const std::vector<VectorXf>& data, float eps, int min_pts) 
        -> std::vector<std::vector<size_t>>;

auto dbscan(const std::vector<std::pair<float, float>>& data, float eps, int min_pts) 
        -> std::vector<std::vector<size_t>>;