#include "path_builder.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <numeric>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>

#include "auxilary.hpp"

using namespace std::chrono_literals;

using namespace Auxilary;

PathBuilder::PathBuilder(float scale_factor) : scale_factor(scale_factor) {}

pcl::PointXYZ PathBuilder::get_point_of_interest(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    auto points_eigen_alloc = cloud->points;
    std::vector<pcl::PointXYZ> points(
        std::make_move_iterator(points_eigen_alloc.begin()),
        std::make_move_iterator(points_eigen_alloc.end()));
    const auto [distances, min_val, mean_val] = get_distances(points, points);

    float max_alpha = 0;
    int max_index = -1;
    auto R_s = linspace(min_val, mean_val, 10);
    float* R_s_data = R_s.data();
    Eigen::VectorXf R_s_eigen =
        Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(R_s_data, R_s.size());
    for (int i = 0; i < cloud->size(); ++i) {
        // const auto dist_point = distances.col(i);
        Eigen::VectorXf dist_r(R_s.size());

        for (int j = 0; j < R_s.size(); ++j) {
            int count = 0;
            float sum_p = 0;

            for (int k = 0; k < distances.rows(); ++k) {
                // for (const auto& p : dist_point.rowwise()) {
                float val = distances(k, i);
                // float val = p(0);
                if (val <= R_s[i]) {
                    ++count;
                    sum_p += val;
                }
            }

            dist_r(j) = count == 0 ? 0 : sum_p / static_cast<float>(count);
        }

        const auto gradients = gradient(dist_r);
        const auto alpha = gradients.mean();

        if (alpha > max_alpha) {
            max_alpha = alpha;
            max_index = i;
        }
    }

    return (*cloud)[max_index];
}

// TODO: Make this function better, use utility functions
void PathBuilder::get_navigation_points(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ& navigate_starting_point,
    const pcl::PointXYZ& known_point1, const pcl::PointXYZ& known_point2,
    const pcl::PointXYZ& known_point3,
    std::vector<pcl::PointXYZ>& path_to_the_unknown, float scale_factor,
    std::vector<pcl::PointXYZ>& RRT_points,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons,
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

    auto [cp, span_v1_gs, span_v2_gs, d] =
        get_plane_from_3_points(known_point1, known_point2, known_point3);

    for (int i = 0; i < 5000; ++i) {
        // pcl::PointXYZ point_rand =
        //     (i % 3 == 0) ? get_random_point_on_plane(cp, d)
        //                  : get_random_point_on_plane(
        //                        navigate_starting_point, *point_of_interest,
        //                        span_v1_gs, span_v2_gs, cp);
        pcl::PointXYZ point_rand =
            // (i % 3 != 0)
            // ? get_random_point_on_plane(cp, d)
            get_random_point_on_plane(span_v1_gs, span_v2_gs);
        // : *point_of_interest;

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
                                  scale_factor, polygons)) {
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

        if (v_dist_closest_point[0] > 2) {
            path_to_the_unknown.clear();
            return;
        }

    } else {
        for (int i = 0; i < dist(generator); ++i) ++n;
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

    // auto start = std::chrono::steady_clock::now();
    // auto something = recursive_robust_median_clustering(cloud, 20);
    // auto end = std::chrono::steady_clock::now();

    // std::cout
    //     << "Elapsed time in seconds: "
    //     << std::chrono::duration_cast<std::chrono::seconds>(end -
    //     start).count()
    //     << " sec";

    auto cluster_indices = get_clusters(cloud);
    auto convexhulls = get_convexhulls(cloud, cluster_indices);
    std::size_t tries = 30;
    // std::shared_ptr<pcl::PointXYZ> poi(
    //     new pcl::PointXYZ(get_point_of_interest(cloud)));

    // while (path_to_the_unknown.size() < how_long_valid_path && tries-- > 0) {
    while (RRT_points.size() < 5 && tries-- > 0) {
        std::cout << "NOT VALID" << std::endl;
        static std::size_t tries_scale_factor_until_change = tries_scale_factor;

        path_to_the_unknown.clear();
        RRT_points.clear();

        get_navigation_points(cloud, start_point, known_point1, known_point2,
                              known_point3, path_to_the_unknown, scale_factor,
                              RRT_points, cluster_indices, convexhulls,
                              nullptr);

        // if (tries_scale_factor_until_change == 0) {
        //     scale_factor = scale_factor * (1 - change_scale_factor);

        //     tries_scale_factor_until_change = tries_scale_factor;
        // }

        // --tries_scale_factor_until_change;

        if (tries % 10 == 0) {
            cluster_indices = get_clusters(cloud);
            convexhulls = get_convexhulls(cloud, cluster_indices);
        }
    }

    // if (path_to_the_unknown.size() < how_long_valid_path) {
    //     path_to_the_unknown.clear();
    // }

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
