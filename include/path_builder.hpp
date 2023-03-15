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
    const std::size_t how_long_valid_path = 5;
    const int rrt_graph_max_size = 3000;

    bool debug = true;

    /**
     * @brief Run DFS to find all paths to leaves in a graph
     */
    static std::vector<std::vector<pcl::PointXYZ>> get_all_paths_to_leaves(
        const lemon::ListDigraph& graph,
        const lemon::ListDigraph::NodeMap<pcl::PointXYZ>& node_map,
        const lemon::ListDigraph::Node& start_node);

    /**
     * @brief Returns the mean of distances from the closest k polygons for
     * every node in the path
     * @returns a pair containing a vector of the mean distances, vector of
     * polygon indices
     */
    static std::pair<std::vector<double>, std::vector<std::vector<std::size_t>>>
    get_top_k_polygon_distances(
        const std::vector<pcl::PointXYZ>& path,
        const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons,
        int k = 10);

    // NOTE: We don't use this function at the moment, but it might be used
    /**
     * @brief Gets the index where the polygon indices stop changing
     */
    static std::size_t get_last_change(
        const std::vector<std::vector<std::size_t>>& polygon_idxs);

    /**
     * @brief Implementation taken from
     * https://github.com/numpy/numpy-financial/blob/main/numpy_financial/_financial.py
     */
    static Eigen::VectorXd g_div_gp(const Eigen::VectorXd& fv,
                                    const Eigen::VectorXd& pv,
                                    const Eigen::VectorXd& rn, int w, int nper,
                                    int pmt);

    /**
     * @brief Get growth rate of distances from polygons
     */
    static double get_path_rate(const std::vector<double>& distances);

    /**
     * @brief Split distances into monotonic parts
     */
    static std::vector<std::vector<double>> split_distances(
        const std::vector<double>& distances);

    // NOTE: This function is not used at the moment, delete?
    static pcl::PointXYZ get_point_of_interest(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    /**
     * @brief Run the RRT algorithm and get the best path to navigate
     * @param known_point[1-3] - points for plane calculation
     * @param polygons - polygons we use as obstacles for RRT
     * @param RRT_points - all points of RRT
     */
    void get_navigation_points(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& navigate_starting_point,
        const pcl::PointXYZ& known_point1, const pcl::PointXYZ& known_point2,
        const pcl::PointXYZ& known_point3,
        std::vector<pcl::PointXYZ>& path_to_the_unknown,
        std::vector<pcl::PointXYZ>& RRT_points,
        const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons);

    /**
     * @brief Find the best path in the RRT graph
     */
    std::vector<pcl::PointXYZ> find_best_path(
        const lemon::ListDigraph& graph,
        const lemon::ListDigraph::NodeMap<pcl::PointXYZ>& node_map,
        const lemon::ListDigraph::Node& start_node,
        const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons);

   public:
    std::vector<std::vector<pcl::PointXYZ>> best_paths;

    /**
     * @brief Get path to the unknown
     * @param cloud - entire model
     * @param start_point - start the RRT from this point
     * @param known_point[1-3] - points for plane calculation
     */
    std::vector<pcl::PointXYZ> operator()(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
        const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3,
        std::vector<pcl::PointXYZ>& RRT_graph);

    void operator()(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                    const pcl::PointXYZ& start_point,
                    const pcl::PointXYZ& known_point1,
                    const pcl::PointXYZ& known_point2,
                    const pcl::PointXYZ& known_point3,
                    std::vector<pcl::PointXYZ>& path_to_the_unknown,
                    std::vector<pcl::PointXYZ>& RRT_graph);

    void operator()(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
        const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3,
        const std::filesystem::path& location_file_path_to_the_unknown,
        std::vector<pcl::PointXYZ>& RRT_graph);
};

#endif  // PATH_BUILDER_H_
