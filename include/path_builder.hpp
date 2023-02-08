#ifndef PATH_BUILDER_H_
#define PATH_BUILDER_H_

#include <pcl/point_cloud.h>

#include <cstddef>
#include <filesystem>
#include <pcl/impl/point_types.hpp>
#include <vector>

class PathBuilder {
    float scale_factor = 0.2;
    const float change_scale_factor = 0.1;
    const std::size_t tries_scale_factor = 10;
    const std::size_t how_long_valid_path = 10;

    bool debug = true;

   public:
    explicit PathBuilder(float scale_factor);
    PathBuilder();

    static void get_navigation_points(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& navigate_starting_point,
        const pcl::PointXYZ& known_point1, const pcl::PointXYZ& known_point2,
        const pcl::PointXYZ& known_point3,
        std::vector<pcl::PointXYZ>& path_to_the_unknown, float scale_factor);

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
        const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3);

    /**
     * @brief This function takes parameters and
     * return the path to the unknown via path_to_the_unknown
     * variable
     * @param cloud[in] -> Entire model
     * @param StartPoint[in] -> Start Point
     * @param knownPoint1[in] -> Points needed for creating the plane for RRT
     * @param path_to_the_unknown[out] -> The returned path to the unknown
     * */
    void operator()(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                    const pcl::PointXYZ& start_point,
                    const pcl::PointXYZ& known_point1,
                    const pcl::PointXYZ& known_point2,
                    const pcl::PointXYZ& known_point3,
                    std::vector<pcl::PointXYZ>& path_to_the_unknown);
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
        const std::filesystem::path& location_file_path_to_the_unknown);
};

#endif  // PATH_BUILDER_H_
