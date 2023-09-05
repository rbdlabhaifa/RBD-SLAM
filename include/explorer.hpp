#ifndef EXPLORER_H_
#define EXPLORER_H_

#include <pcl/point_cloud.h>

#include <array>
#include <pcl/impl/point_types.hpp>
#include <vector>

class Explorer
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::array<pcl::PointXYZ, 3> known_points;
    std::vector<pcl::PointXYZ> RRT_points;
    bool got_plane_of_flight = false;

  public:
    /// Keeps k best paths we found
    std::vector<std::vector<pcl::PointXYZ>> best_paths;

    explicit Explorer(
        const std::vector<Eigen::Matrix<double, 3, 1>> &map_points);

    explicit Explorer(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    /**
     * @brief Add points from ORB_SLAM to the internal point cloud
     */
    void add_points_to_cloud(
        const std::vector<Eigen::Matrix<double, 3, 1>> &map_points);
    std::vector<pcl::PointXYZ> get_last_graph();

    void set_cloud_points(
        const std::vector<Eigen::Matrix<double, 3, 1>> &map_points);

    void set_plane_of_flight(const pcl::PointXYZ &known_point1,
                             const pcl::PointXYZ &known_point2,
                             const pcl::PointXYZ &known_point3);

    bool is_set_plane_of_flight() const { return got_plane_of_flight; }

    // NOTE: The scale_factor is not used at the moment
    /**
     * @brief Use the path builder API to get a path to the unknown
     */
    std::vector<pcl::PointXYZ>
    get_points_to_unknown(const pcl::PointXYZ &start_point);
};

#endif // EXPLORER_H_
