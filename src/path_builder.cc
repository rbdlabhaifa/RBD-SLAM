#include "path_builder.hpp"

#include <lemon/bfs.h>
#include <lemon/core.h>
#include <lemon/list_graph.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <random>
#include <vector>

#include "auxilary.hpp"

using namespace Auxilary;

PathBuilder::PathBuilder(float scale_factor) : scale_factor(scale_factor) {}
PathBuilder::PathBuilder() {}

// TODO: Make this function better, use utility functions
void PathBuilder::get_navigation_points(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ& navigate_starting_point,
    const pcl::PointXYZ& known_point1, const pcl::PointXYZ& known_point2,
    const pcl::PointXYZ& known_point3,
    std::vector<pcl::PointXYZ>& path_to_the_unknown, float scale_factor) {
    lemon::ListGraph RRT_graph;
    lemon::ListGraph::NodeMap<pcl::PointXYZ> node_to_point(RRT_graph);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<Edge> v_edges;

    pcl::PointCloud<pcl::PointXYZ>::Ptr navigate_data(
        new pcl::PointCloud<pcl::PointXYZ>);

    navigate_data->push_back(navigate_starting_point);
    lemon::ListGraph::Node first_node = RRT_graph.addNode();

    node_to_point[first_node] = navigate_starting_point;

    float jump = 0.8 * scale_factor;

    for (int i = 0; i < 2000; ++i) {
        pcl::PointXYZ point_rand = get_random_point_on_plane_def_by_3_points(
            known_point1, known_point2, known_point3);

        pcl::KdTreeFLANN<pcl::PointXYZ> navigate_tree;
        navigate_tree.setInputCloud(navigate_data);

        std::vector<int> v_closest_point(1);
        std::vector<float> v_dist_closest_point(1);

        if (navigate_tree.nearestKSearch(point_rand, 1, v_closest_point,
                                         v_dist_closest_point)) {
            const pcl::PointXYZ& close_point =
                (*navigate_data)[v_closest_point[0]];

            pcl::PointXYZ point_new =
                close_point + jump * (point_rand - close_point) /
                                  (sqrt((point_rand - close_point) *
                                        (point_rand - close_point)));

            if (is_valid_movement(cloud, close_point, point_new, kdtree,
                                  scale_factor)) {
                navigate_data->push_back(point_new);
                v_edges.push_back({point_new, close_point});

                lemon::ListGraph::Node new_node = RRT_graph.addNode();
                node_to_point[new_node] = point_new;

                lemon::ListGraph::Node closest_point_RRT;

                for (lemon::ListGraph::NodeIt it(RRT_graph);
                     it != lemon::INVALID; ++it) {
                    if (node_to_point[it] == close_point)
                        closest_point_RRT = it;
                }

                lemon::ListGraph::Edge new_edge =
                    RRT_graph.addEdge(new_node, closest_point_RRT);
            }
        } else {
            std::cout << "Cannot find Nearest Node" << std::endl;
        }
    }

    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();

    if (lemon::countNodes(RRT_graph) < 50) return;

    std::random_device os_seed;
    const uint_least32_t seed = os_seed();

    std::mt19937 generator(seed);
    std::uniform_int_distribution<uint_least32_t> dist(
        1, lemon::countNodes(RRT_graph) - 2);
    lemon::ListGraph::NodeIt n(RRT_graph);

    for (int i = 0; i < dist(generator); ++i) ++n;

    lemon::ListGraph::Node get_random_node_rrt = n;

    // BFS
    lemon::Bfs<lemon::ListGraph> bfs(RRT_graph);
    bfs.init();
    bfs.addSource(first_node);
    bfs.start();

    if (bfs.reached(get_random_node_rrt)) {
        path_to_the_unknown.insert(path_to_the_unknown.begin(),
                                   node_to_point[get_random_node_rrt]);

        lemon::ListGraph::Node prev = bfs.predNode(get_random_node_rrt);

        while (prev != lemon::INVALID) {
            path_to_the_unknown.insert(path_to_the_unknown.begin(),
                                       node_to_point[prev]);
            prev = bfs.predNode(prev);
        }
    }

    if (path_to_the_unknown.size() < 10) return;
}

std::vector<pcl::PointXYZ> PathBuilder::operator()(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
    const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3) {
    std::vector<pcl::PointXYZ> path_to_the_unknown;

    while (path_to_the_unknown.size() < how_long_valid_path) {
        static std::size_t tries_scale_factor_until_change = tries_scale_factor;

        path_to_the_unknown.clear();

        get_navigation_points(cloud, start_point, known_point1, known_point2,
                              known_point3, path_to_the_unknown, scale_factor);

        if (tries_scale_factor_until_change == 0) {
            scale_factor = scale_factor * (1 - change_scale_factor);

            tries_scale_factor_until_change = tries_scale_factor;
        }

        --tries_scale_factor_until_change;
    }

    if (debug)
        for (auto point : path_to_the_unknown)
            std::cout << point.x << " " << point.y << " " << point.z << "->";

    return path_to_the_unknown;
}

void PathBuilder::operator()(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                             const pcl::PointXYZ& start_point,
                             const pcl::PointXYZ& known_point1,
                             const pcl::PointXYZ& known_point2,
                             const pcl::PointXYZ& known_point3,
                             std::vector<pcl::PointXYZ>& path_to_the_unknown) {
    path_to_the_unknown = operator()(cloud, start_point, known_point1,
                                     known_point2, known_point3);
}

void PathBuilder::operator()(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
    const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3,
    const std::filesystem::path& location_file_path_to_the_unknown) {
    std::vector<pcl::PointXYZ> path_to_the_unknown = operator()(
        cloud, start_point, known_point1, known_point2, known_point3);
    save_path_to_file(path_to_the_unknown, location_file_path_to_the_unknown);
}
