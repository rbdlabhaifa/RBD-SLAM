#ifndef AUXILARY_H_
#define AUXILARY_H_

#include <libqhullcpp/QhullFacetList.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include <array>
#include <boost/container_hash/hash.hpp>
#include <cstddef>
#include <filesystem>
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

    std::vector<ConvexHullEquations> get_convexhulls(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const std::vector<pcl::PointIndices>& cluster_indices);

    pcl::PointXYZ vec_to_pcl(std::vector<double> vec);

    bool is_valid_movement(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                           const pcl::PointXYZ& current_point,
                           const pcl::PointXYZ& dest_point,
                           const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                           float scale_factor,
                           const std::vector<ConvexHullEquations>& convexhulls);

    bool check_convexhull_intersection(const pcl::PointXYZ& p,
                                       const ConvexHullEquations& convexhull,
                                       double tolerance = 1e-12);

    bool check_convexhull_intersection(const pcl::PointXYZ& start,
                                       const pcl::PointXYZ& end,
                                       const ConvexHullEquations& convexhull);

    std::pair<pcl::PointXYZ, float> get_plane_from_3_points(
        const pcl::PointXYZ& p1, const pcl::PointXYZ& p2,
        const pcl::PointXYZ& p3);
    pcl::PointXYZ get_random_point_on_plane(const pcl::PointXYZ& cp, float d);

    std::vector<pcl::PointIndices> get_clusters(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree);

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
