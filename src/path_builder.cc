#include "path_builder.hpp"

#include <libqhullcpp/Qhull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <unordered_map>
#include <vector>

#include "auxilary.hpp"

using namespace Auxilary;

PathBuilder::PathBuilder(float scale_factor) : scale_factor(scale_factor) {}

// TODO: Make this function better, use utility functions
void PathBuilder::get_navigation_points(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ& navigate_starting_point,
    const pcl::PointXYZ& known_point1, const pcl::PointXYZ& known_point2,
    const pcl::PointXYZ& known_point3,
    std::vector<pcl::PointXYZ>& path_to_the_unknown, float scale_factor,
    std::vector<pcl::PointXYZ>& RRT_points,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const std::vector<Auxilary::ConvexHullEquations>& convexhulls,
    const std::shared_ptr<pcl::PointXYZ>& point_of_interest) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr RRT_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    lemon::ListGraph RRT_graph;
    lemon::ListGraph::NodeMap<pcl::PointXYZ> node_to_point(RRT_graph);
    auto comp_func = [](const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
    };
    auto hash_func = [](const pcl::PointXYZ& p) {
        std::size_t seed = 0;
        boost::hash_combine(seed, p.x);
        boost::hash_combine(seed, p.y);
        boost::hash_combine(seed, p.z);
        return seed;
    };
    std::unordered_map<pcl::PointXYZ, lemon::ListGraph::Node,
                       decltype(hash_func), decltype(comp_func)>
        point_to_node(8, hash_func, comp_func);

    std::vector<Edge> v_edges;

    pcl::PointCloud<pcl::PointXYZ>::Ptr navigate_data(
        new pcl::PointCloud<pcl::PointXYZ>);

    navigate_data->push_back(navigate_starting_point);
    lemon::ListGraph::Node first_node = RRT_graph.addNode();

    node_to_point[first_node] = navigate_starting_point;
    point_to_node[navigate_starting_point] = first_node;

    float jump = 0.2;
    // float jump = 0.2 * scale_factor;
    pcl::KdTreeFLANN<pcl::PointXYZ> navigate_tree;

    auto [cp, d] =
        get_plane_from_3_points(known_point1, known_point2, known_point3);

    for (int i = 0; i < 5000; ++i) {
        pcl::PointXYZ point_rand = get_random_point_on_plane(cp, d);

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
                                  scale_factor, convexhulls)) {
                navigate_data->push_back(point_new);
                v_edges.push_back({point_new, close_point});

                lemon::ListGraph::Node new_node = RRT_graph.addNode();
                node_to_point[new_node] = point_new;
                point_to_node[point_new] = new_node;

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

    std::vector<int> v_closest_point(1);
    std::vector<float> v_dist_closest_point(1);

    if (point_of_interest != nullptr) {
        std::cout << *point_of_interest << std::endl;
    }

    if (point_of_interest != nullptr) {
        navigate_tree.nearestKSearch(
            (*point_of_interest - navigate_starting_point) / 2, 1,
            v_closest_point, v_dist_closest_point);
    } else {
        if (point_of_interest == nullptr) {
            for (int i = 0; i < dist(generator); ++i) ++n;
        }
    }

    lemon::ListGraph::Node get_random_node_rrt =
        point_of_interest == nullptr
            ? n
            : point_to_node[(*navigate_data)[v_closest_point[0]]];

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

    for (lemon::ListGraph::NodeIt it(RRT_graph); it != lemon::INVALID; ++it) {
        RRT_points.push_back(node_to_point[it]);
    }

    std::cout << node_to_point[get_random_node_rrt] << std::endl;
}

std::vector<pcl::PointXYZ> PathBuilder::operator()(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
    const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3,
    std::vector<pcl::PointXYZ>& RRT_points,
    const std::shared_ptr<pcl::PointXYZ>& point_of_interest) {
    std::vector<pcl::PointXYZ> path_to_the_unknown;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_search(
        new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree_search->setInputCloud(cloud);
    auto cluster_indices = get_clusters(cloud, kdtree_search);
    const auto convexhulls = get_convexhulls(cloud, cluster_indices);

    while (path_to_the_unknown.size() < how_long_valid_path) {
        std::cout << "NOT VALID" << std::endl;
        static std::size_t tries_scale_factor_until_change = tries_scale_factor;

        path_to_the_unknown.clear();

        get_navigation_points(cloud, start_point, known_point1, known_point2,
                              known_point3, path_to_the_unknown, scale_factor,
                              RRT_points, cluster_indices, convexhulls,
                              point_of_interest);

        // if (tries_scale_factor_until_change == 0) {
        //     scale_factor = scale_factor * (1 - change_scale_factor);

        //     tries_scale_factor_until_change = tries_scale_factor;
        // }

        // --tries_scale_factor_until_change;
    }

    if (debug)
        for (auto point : path_to_the_unknown)
            std::cout << point.x << " " << point.y << " " << point.z << "->";

    return path_to_the_unknown;
}

void PathBuilder::operator()(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
    const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3,
    std::vector<pcl::PointXYZ>& path_to_the_unknown,
    std::vector<pcl::PointXYZ>& RRT_points,
    const std::shared_ptr<pcl::PointXYZ>& point_of_interest) {
    path_to_the_unknown = operator()(cloud, start_point, known_point1,
                                     known_point2, known_point3, RRT_points,
                                     point_of_interest);
}

void PathBuilder::operator()(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
    const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3,
    const std::filesystem::path& location_file_path_to_the_unknown,
    std::vector<pcl::PointXYZ>& RRT_points,
    const std::shared_ptr<pcl::PointXYZ>& point_of_interest) {
    std::vector<pcl::PointXYZ> path_to_the_unknown = operator()(
        cloud, start_point, known_point1, known_point2, known_point3,
        RRT_points, point_of_interest);
    save_points_to_file(path_to_the_unknown, location_file_path_to_the_unknown);
}
