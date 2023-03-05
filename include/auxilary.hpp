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
#include <opencv2/core/types.hpp>
#include <pcl/impl/point_types.hpp>
#include <vector>

namespace Auxilary {
    struct FacetPlane {
        pcl::PointXYZ normal;
        double offset = 0;
    };
    struct ConvexHullEquations {
        std::vector<FacetPlane> facets_planes;
    };

    struct Edge {
        pcl::PointXYZ p1;
        pcl::PointXYZ p2;
    };

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

    pcl::PointXYZ operator-(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);
    pcl::PointXYZ operator/(const pcl::PointXYZ& p, float d);

    float norm(const pcl::PointXYZ& p);

    /**
     * @brief Given 2 vectors of size 3*1 and 3*1
     * returns the matrix multipication
     * @param pcl::PointXYZ p1[in] -> is the first point
     * @param pcl::PointXYZ p2[in] -> is the second point
     * @return float -> The matrix mul (1*3)*(3*1)
     */
    float operator*(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

    pcl::PointXYZ operator*(const pcl::PointXYZ& p, float a);
    pcl::PointXYZ operator*(float a, const pcl::PointXYZ& p);
    pcl::PointXYZ operator+(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

    bool operator==(const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs);

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

    std::vector<std::unique_ptr<geos::geom::Geometry>> get_convexhulls(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const std::vector<pcl::PointIndices>& cluster_indices);

    std::vector<std::unique_ptr<geos::geom::Geometry>> get_polygons(
        const std::vector<std::vector<pcl::PointXYZ>>& points);

    pcl::PointXYZ vec_to_pcl(std::vector<double> vec);

    bool is_valid_movement(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& current_point, const pcl::PointXYZ& dest_point,
        const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, float scale_factor,
        const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons);

    bool check_convexhull_intersection(const pcl::PointXYZ& p,
                                       const ConvexHullEquations& convexhull,
                                       double tolerance = 1e-12);

    bool check_polygon_intersection(
        const pcl::PointXYZ& start, const pcl::PointXYZ& end,
        const std::unique_ptr<geos::geom::Geometry>& polygon);

    bool check_convexhull_intersection(const pcl::PointXYZ& start,
                                       const pcl::PointXYZ& end,
                                       const ConvexHullEquations& convexhull);

    template <typename T>
    std::vector<T> linspace(T a, T b, std::size_t N) {
        T h = (b - a) / static_cast<T>(N - 1);
        std::vector<T> xs(N);
        typename std::vector<T>::iterator x;
        T val;
        for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) *x = val;
        return xs;
    }

    Eigen::VectorXf gradient(const Eigen::VectorXf& v);

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
    std::vector<std::size_t> argsort(const std::vector<T>& vec);

    template <typename T>
    void remove_row(Eigen::Matrix<T, Eigen::Dynamic, 1>& matrix,
                    Eigen::Index rowToRemove);

    std::vector<float> sum_rows(const Eigen::VectorXf& mat);

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> get_all_indices_except(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& vec,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices);

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> get_indices(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& vec,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices);

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> get_indices(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& vec,
        const std::vector<std::size_t>& indices);

    template <typename T>
    std::vector<T> get_indices(
        const std::vector<T>& vec,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices);

    void delaunay_greedy_based_picking(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    std::vector<pcl::PointXYZ> recursive_robust_median_clustering(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int k);

    void sort_matrix_columns(Eigen::MatrixXf& mat);

    pcl::PointXYZ robust_median(const std::vector<pcl::PointXYZ>& points,
                                int k);

    std::vector<std::size_t> get_random_indices(std::size_t vec_length,
                                                std::size_t amount);

    std::vector<pcl::PointIndices> get_clusters(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    pcl::PointXYZ cross_product(const pcl::PointXYZ& v_a,
                                const pcl::PointXYZ& v_b);

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
