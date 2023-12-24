// Credits: https://github.com/Eleobert/dbscan

#include <Eigen/Dense>
#include <vector>

auto dbscanVectorXf(const std::vector<VectorXf> &data, float eps, int min_pts)
    -> std::vector<std::vector<unsigned int>>;

auto dbscan(const std::vector<std::pair<float, float>> &data, float eps,
            int min_pts) -> std::vector<std::vector<unsigned int>>;
