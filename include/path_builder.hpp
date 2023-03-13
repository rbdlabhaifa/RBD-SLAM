#ifndef PATH_BUILDER_H_
#define PATH_BUILDER_H_

#include <geos/geom/Geometry.h>
#include <lemon/bfs.h>
#include <lemon/core.h>
#include <lemon/dfs.h>
#include <lemon/list_graph.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <pcl/impl/point_types.hpp>
#include <vector>

#include "auxilary.hpp"

class PathBuilder {
    float scale_factor;
    const float change_scale_factor = 0.1;
    const std::size_t tries_scale_factor = 10;
    const std::size_t how_long_valid_path = 5;
    const int rrt_graph_max_size = 3000;

    bool debug = true;

    static std::vector<std::vector<pcl::PointXYZ>> get_all_paths_to_leaves(
        const lemon::ListDigraph& graph,
        const lemon::ListDigraph::NodeMap<pcl::PointXYZ>& node_map,
        const lemon::ListDigraph::Node& start_node);

    static std::pair<std::vector<double>, std::vector<std::vector<std::size_t>>>
    get_top_k_polygon_distances(
        const std::vector<pcl::PointXYZ>& path,
        const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons,
        int k = 10);

    static std::size_t get_last_change(
        const std::vector<std::vector<std::size_t>>& polygon_idxs);

    static Eigen::VectorXd g_div_gp(const Eigen::VectorXd& fv,
                                    const Eigen::VectorXd& pv,
                                    const Eigen::VectorXd& rn, int w, int nper,
                                    int pmt);
    static double get_path_rate(const std::vector<double>& distances);
    static double get_path_rate(const std::vector<pcl::PointXYZ>& path);
    static std::vector<std::vector<double>> split_distances(
        const std::vector<double>& distances);

    static pcl::PointXYZ get_point_of_interest(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    void get_navigation_points(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& navigate_starting_point,
        const pcl::PointXYZ& known_point1, const pcl::PointXYZ& known_point2,
        const pcl::PointXYZ& known_point3,
        std::vector<pcl::PointXYZ>& path_to_the_unknown, float scale_factor,
        std::vector<pcl::PointXYZ>& RRT_points,
        const std::vector<pcl::PointIndices>& cluster_indices,
        const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons);

    std::vector<pcl::PointXYZ> find_best_path(
        const lemon::ListDigraph& graph,
        const lemon::ListDigraph::NodeMap<pcl::PointXYZ>& node_map,
        const lemon::ListDigraph::Node& start_node,
        const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons);

   public:
    explicit PathBuilder(float scale_factor = 0.2);
    std::vector<std::vector<pcl::PointXYZ>> best_paths;

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
