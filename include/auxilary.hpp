#ifndef AUXILARY_H_
#define AUXILARY_H_

#include <geos/geom/Geometry.h>
#include <geos/geom/Polygon.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include <array>
#include <boost/container_hash/hash.hpp>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <numeric>
#include <opencv2/core/types.hpp>
#include <pcl/impl/point_types.hpp>
#include <vector>

namespace Auxilary {
    /**
     * @brief This function search for additional points in the radius
     * of "PointXYZ searchPoint"
     * @param cloud -> The Cloud
     * @param searchPoint -> the point I am searching around
     * @param radius -> The radius of search
     * @return -> number of points that are in the neigbourhood of searchPoint
     */
    int radius_search(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                      const pcl::PointXYZ& search_point, float radius,
                      const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree);

    // Hashable function for pcl::PointXYZ
    /**
     * @brief
     * This function build a line between @var start and @var end
     * and return all inside the line with distance at least
     * @var jump_distance
     * @param pcl::PointXYZ start -> 1 of the points to create a line
     * @param pcl::PointXYZ end -> the other point to create the line
     * @param float jump_distance -> sets the minimum distance of the point on
     * line
     * @return -> all points on line with distance at least @var jump_distance
     */
    std::vector<pcl::PointXYZ> get_points_on_line(const pcl::PointXYZ& start,
                                                  const pcl::PointXYZ& end,
                                                  float jump_distance);

    std::vector<pcl::PointIndices> get_clusters(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::size_t minimum_cluster_size = 60);

    void save_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                       const std::vector<pcl::PointIndices>& cluster_indices);

    std::vector<std::unique_ptr<geos::geom::Geometry>> get_convexhulls(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const std::vector<pcl::PointIndices>& cluster_indices);

    bool is_valid_movement(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& current_point, const pcl::PointXYZ& dest_point,
        const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, float scale_factor,
        const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons);

    bool check_polygon_intersection(
        const pcl::PointXYZ& start, const pcl::PointXYZ& end,
        const std::unique_ptr<geos::geom::Geometry>& polygon);

    template <typename T>
    std::vector<T> linspace(T a, T b, std::size_t N) {
        T h = (b - a) / static_cast<T>(N - 1);
        std::vector<T> xs(N);
        typename std::vector<T>::iterator x;
        T val;
        for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) *x = val;
        return xs;
    }

    std::tuple<Eigen::MatrixXf, float, float> get_distances(
        const std::vector<pcl::PointXYZ>& v1,
        const std::vector<pcl::PointXYZ>& v2);

    std::tuple<pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ, float>
    get_plane_from_3_points(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2,
                            const pcl::PointXYZ& p3);

    pcl::PointXYZ get_random_point_on_plane(const pcl::PointXYZ& cp, float d);
    pcl::PointXYZ get_random_point_on_plane(const pcl::PointXYZ& span_v1,
                                            const pcl::PointXYZ& span_v2);
    pcl::PointXYZ get_random_point_on_plane(
        const pcl::PointXYZ& start, const pcl::PointXYZ& point_of_interest,
        const pcl::PointXYZ& u1, const pcl::PointXYZ& u2,
        const pcl::PointXYZ& cp);

    template <typename T>
    std::vector<std::size_t> argsort(const std::vector<T>& vec,
                                     bool reverse = false) {
        std::vector<size_t> indices(vec.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(),
                  [&vec, &reverse](auto left, auto right) {
                      return reverse ? vec[left] > vec[right]
                                     : vec[left] < vec[right];
                  });

        return indices;
    }

    std::vector<pcl::PointXYZ> recursive_robust_median_clustering(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int k);

    pcl::PointXYZ robust_median(const std::vector<pcl::PointXYZ>& points,
                                int k);

    std::vector<std::size_t> get_random_indices(std::size_t vec_length,
                                                std::size_t amount);

    void save_points_to_file(const std::vector<pcl::PointXYZ>& points,
                             const std::filesystem::path& location_file_path);
    void save_points_to_file(
        const std::vector<Eigen::Matrix<double, 3, 1>>& points,
        const std::filesystem::path& location_file_path);
    void save_points_to_file(const std::vector<cv::Point3f>& points,
                             const std::filesystem::path& location_file_path);
    std::vector<pcl::PointXYZ> load_path_from_file(
        const std::filesystem::path& location_file_path);
}  // namespace Auxilary

#endif  // AUXILARY_H_
