#include "explorer.hpp"

#include <vector>

#include "path_builder.hpp"

Explorer::Explorer(const std::vector<Eigen::Matrix<double, 3, 1>> &map_points)
    : cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
    add_points_to_cloud(map_points);
}

Explorer::Explorer(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    : cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud))
{
}

void Explorer::set_cloud_points(
    const std::vector<Eigen::Matrix<double, 3, 1>> &map_points)
{
    cloud->clear();
    add_points_to_cloud(map_points);
}

void Explorer::add_points_to_cloud(
    const std::vector<Eigen::Matrix<double, 3, 1>> &map_points)
{
    for (auto p : map_points)
    {
        cloud->push_back(pcl::PointXYZ(static_cast<float>(p.x()),
                                       static_cast<float>(p.y()),
                                       static_cast<float>(p.z())));
    }
}

void Explorer::set_plane_of_flight(const pcl::PointXYZ &known_point1,
                                   const pcl::PointXYZ &known_point2,
                                   const pcl::PointXYZ &known_point3)
{
    got_plane_of_flight = true;

    known_points = {known_point1, known_point2, known_point3};
}

std::vector<pcl::PointXYZ> Explorer::get_last_graph()
{
    std::vector<pcl::PointXYZ> last_graph = RRT_points;
    RRT_points.clear();
    return last_graph;
}

std::vector<pcl::PointXYZ>
Explorer::get_points_to_unknown(const pcl::PointXYZ &start_point)
{
    if (!got_plane_of_flight)
    {
        std::cerr << "Explorer: Expected plane of flight to be set"
                  << std::endl;
        return {};
    }

    PathBuilder builder;
    auto path = builder(cloud, start_point, known_points[0], known_points[1],
                        known_points[2], RRT_points);
    best_paths = builder.best_paths;

    return path;
}
