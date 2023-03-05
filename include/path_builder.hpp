#ifndef PATH_BUILDER_H_
#define PATH_BUILDER_H_

#include <lemon/bfs.h>
#include <lemon/core.h>
#include <lemon/list_graph.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>

#include <cstddef>
#include <filesystem>
#include <pcl/impl/point_types.hpp>
#include <vector>

#include "auxilary.hpp"

class PathBuilder {
    float scale_factor;
    const float change_scale_factor = 0.1;
    const std::size_t tries_scale_factor = 10;
    const std::size_t how_long_valid_path = 5;

    bool debug = true;
    static pcl::PointXYZ get_point_of_interest(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    static void get_navigation_points(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& navigate_starting_point,
        const pcl::PointXYZ& known_point1, const pcl::PointXYZ& known_point2,
        const pcl::PointXYZ& known_point3,
        std::vector<pcl::PointXYZ>& path_to_the_unknown, float scale_factor,
        std::vector<pcl::PointXYZ>& RRT_graph,
        const std::vector<pcl::PointIndices>& cluster_indices,
        const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons,
        const std::shared_ptr<pcl::PointXYZ>& point_of_interest = nullptr);

   public:
    explicit PathBuilder(float scale_factor = 0.2);

    /**
     * @brief Returns the path to the unknown
     * @param cloud[in] -> Entire model
     * @param StartPoint[in] -> Start Point
     * @param knownPoint1[in] -> Points needed for creating the plane for RRT
     * @returns Path to the unknown
     * */
    std::vector<pcl::PointXYZ> operator()(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
        const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3,
        std::vector<pcl::PointXYZ>& RRT_graph,
        const std::shared_ptr<pcl::PointXYZ>& point_of_interest = nullptr);

    /**
     * @brief This function takes parameters and
     * return the path to the unknown via path_to_the_unknown
     * variable
     * @param cloud[in] -> Entire model
     * @param StartPoint[in] -> Start Point
     * @param knownPoint1[in] -> Points needed for creating the plane for RRT
     * @param path_to_the_unknown[out] -> The returned path to the unknown
     * */
    void operator()(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
        const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3,
        std::vector<pcl::PointXYZ>& path_to_the_unknown,
        std::vector<pcl::PointXYZ>& RRT_graph,
        const std::shared_ptr<pcl::PointXYZ>& point_of_interest = nullptr);
    /**
     * @brief This function takes parameters and
     * return the path to the unknown via path_to_the_unknown
     * variable
     * @override create text file
     * @param cloud[in] -> Entire model
     * @param StartPoint[in] -> Start Point
     * @param knownPoint1[in] -> Points needed for creating the plane for RRT
     * @param path_to_the_unknown[out] -> The returned path to the unknown
     * */
    void operator()(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
        const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3,
        const std::filesystem::path& location_file_path_to_the_unknown,
        std::vector<pcl::PointXYZ>& RRT_graph,
        const std::shared_ptr<pcl::PointXYZ>& point_of_interest = nullptr);
};

#endif  // PATH_BUILDER_H_
