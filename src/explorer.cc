#include "explorer.hpp"

Explorer::Explorer(const std::vector<Eigen::Matrix<double, 3, 1>>& map_points)
    : cloud(new pcl::PointCloud<pcl::PointXYZ>) {
    add_points_to_cloud(map_points);
}

void Explorer::add_points_to_cloud(
    const std::vector<Eigen::Matrix<double, 3, 1>>& map_points) {
    for (auto p : map_points) {
        cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
}
