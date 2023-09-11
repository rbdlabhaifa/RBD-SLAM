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

class PathBuilder
{
    const std::size_t how_long_valid_path = 5;
    const int rrt_graph_max_size = 3000;

    bool debug = true;

    /**
     * @brief Run DFS to find path between two given nodes
     */
    static std::vector<pcl::PointXYZ> get_path_between_two_nodes(
        const lemon::ListDigraph &graph,
        const lemon::ListDigraph::NodeMap<pcl::PointXYZ> &node_map,
        const lemon::ListDigraph::Node &start_node,
        const lemon::ListDigraph::Node &end_node);

    /**
     * @brief Run the RRT algorithm and get the best path to navigate
     * @param known_point[1-3] - points for plane calculation
     * @param polygons - polygons we use as obstacles for RRT
     * @param RRT_points - all points of RRT
     */
    bool get_navigation_points(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ &navigate_starting_point,
        const pcl::PointXYZ &known_point1, const pcl::PointXYZ &known_point2,
        const pcl::PointXYZ &known_point3,
        std::vector<pcl::PointXYZ> &path_to_the_unknown,
        std::vector<pcl::PointXYZ> &RRT_points,
        const std::vector<std::unique_ptr<geos::geom::Geometry>> &polygons,
        const pcl::PointXYZ &goal_point, const float threshold,
        const float jump_size, const int ring_point_amount,
        const float ring_size_scalar);

  public:
    std::vector<std::vector<pcl::PointXYZ>> best_paths;

    std::vector<pcl::PointXYZ> build_path_to_exit(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ &start_point, const pcl::PointXYZ &exit_point,
        const pcl::PointXYZ &known_point1, const pcl::PointXYZ &known_point2,
        const pcl::PointXYZ &known_point3,
        std::vector<pcl::PointXYZ> &RRT_graph, const float threshold,
        const float jump_size, const int ring_point_amount,
        const float ring_size_scalar);
};

#endif // PATH_BUILDER_H_
