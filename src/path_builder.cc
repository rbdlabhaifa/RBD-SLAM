#include "path_builder.hpp"

#include "auxilary.hpp"
#include "eigen_operations.hpp"
#include "pcl_operations.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <geos/geom/GeometryFactory.h>
#include <iostream>
#include <iterator>
#include <lemon/core.h>
#include <lemon/list_graph.h>
#include <lemon/path.h>
#include <memory>
#include <numeric>
#include <opencv2/core/mat.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
using namespace Auxilary;
using namespace PCLOperations;
using namespace EigenOperations;

std::vector<pcl::PointXYZ> PathBuilder::get_path_between_two_nodes(
    const lemon::ListDigraph &graph,
    const lemon::ListDigraph::NodeMap<pcl::PointXYZ> &node_map,
    const lemon::ListDigraph::Node &start_node,
    const lemon::ListDigraph::Node &end_node)
{
    std::vector<pcl::PointXYZ> path; // Initialize an empty path vector.
    lemon::ListDigraph::Node current_node = end_node;

    while (current_node != start_node)
    {
        path.push_back(node_map[current_node]);
        for (lemon::ListDigraph::InArcIt it(graph, current_node);
             it != lemon::INVALID; ++it)
        {
            current_node = graph.source(it);
            break; // We assume there's only one parent node.
        }
    }
    // Add the start node's point (the root) to the path.
    path.push_back(node_map[start_node]);
    // Reverse the order of points to get the path from root to end.
    std::reverse(path.begin(), path.end());

    return path;
}

bool PathBuilder::get_navigation_points(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ &navigate_starting_point,
    const pcl::PointXYZ &known_point1, const pcl::PointXYZ &known_point2,
    const pcl::PointXYZ &known_point3,
    std::vector<pcl::PointXYZ> &path_to_the_unknown,
    std::vector<pcl::PointXYZ> &RRT_points,
    const std::vector<std::unique_ptr<geos::geom::Geometry>> &polygons,
    const pcl::PointXYZ &goal_point, const float threshold,
    const float jump_size, const int ring_point_amount,
    const float ring_size_scalar)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr RRT_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    lemon::ListDigraph RRT_graph;
    lemon::ListDigraph::NodeMap<pcl::PointXYZ> node_to_point(RRT_graph);

    pcl::PointCloud<pcl::PointXYZ>::Ptr navigate_data(
        new pcl::PointCloud<pcl::PointXYZ>);

    navigate_data->push_back(navigate_starting_point);
    lemon::ListDigraph::Node first_node = RRT_graph.addNode();
    auto map_filename = "start.xyz";
    cv::Point3f navigateCv =
        cv::Point3f(navigate_starting_point.x, navigate_starting_point.y,
                    navigate_starting_point.z);
    Auxilary::save_points_to_file({navigateCv}, map_filename);

    node_to_point[first_node] = navigate_starting_point;

    pcl::KdTreeFLANN<pcl::PointXYZ> navigate_tree;
    pcl::PointXYZ plane_mean = (known_point1 + known_point2 + known_point3) / 3;

    auto [cp, span_v1_gs, span_v2_gs, d] = get_plane_from_3_points(
        known_point1 - plane_mean, known_point2 - plane_mean,
        known_point3 - plane_mean);
    // shershor for the A's to calculate the center
    cv::Mat A;
    A = cv::Mat::zeros(2, 3, CV_32F);
    A.at<float>(0, 0) = span_v1_gs.x;
    A.at<float>(0, 1) = span_v1_gs.y;
    A.at<float>(0, 2) = span_v1_gs.z;
    A.at<float>(1, 0) = span_v2_gs.x;
    A.at<float>(1, 1) = span_v2_gs.y;
    A.at<float>(1, 2) = span_v2_gs.z;

    cv::Mat X = cv::Mat::zeros(1, 3, CV_32F);
    cv::Mat Ainv = cv::Mat::zeros(3, 2, CV_32F);

    invert(A, Ainv, cv::DECOMP_SVD);

    X.at<float>(0, 0) = navigate_starting_point.x;
    X.at<float>(0, 1) = navigate_starting_point.y;
    X.at<float>(0, 2) = navigate_starting_point.z;
    cv::Mat Center = cv::Mat::zeros(1, 2, CV_32F);

    Center = X * Ainv;
    int rrt_size = 0;
    std::vector<float> Cntr{Center.at<float>(0, 0), Center.at<float>(0, 1),
                            Center.at<float>(0, 2)};

    bool stop_rrt = false;
    lemon::ListDigraph::Node goal_node;
    double min_dist_from_goal = 100;

    for (int i = 0; i < rrt_graph_max_size; ++i)
    {
        if (stop_rrt)
        {
            break;
        }
        float R = ((i / ring_point_amount) + 0.5) * ring_size_scalar;
        float r = (i / ring_point_amount) * ring_size_scalar;
        pcl::PointXYZ point_rand =
            get_random_point_on_plane(span_v1_gs, span_v2_gs, Cntr, R, r) +
            plane_mean;

        navigate_tree.setInputCloud(navigate_data);

        std::vector<int> v_closest_point(1);
        std::vector<float> v_dist_closest_point(1);

        if (navigate_tree.nearestKSearch(point_rand, 1, v_closest_point,
                                         v_dist_closest_point) > 0)
        {
            const pcl::PointXYZ &close_point =
                (*navigate_data)[v_closest_point[0]];

            pcl::PointXYZ point_new =
                close_point + jump_size * (point_rand - close_point) /
                                  (std::sqrt((point_rand - close_point) *
                                             (point_rand - close_point)));

            if (is_valid_movement(cloud, close_point, point_new, polygons))
            {
                for (lemon::ListDigraph::NodeIt it(RRT_graph);
                     it != lemon::INVALID; ++it)
                {
                    if (node_to_point[it] == close_point)
                    {
                        navigate_data->push_back(point_new);
                        lemon::ListDigraph::Node new_node = RRT_graph.addNode();
                        node_to_point[new_node] = point_new;
                        RRT_graph.addArc(it, new_node);
                        ++rrt_size;
                        double dist_from_goal = norm(point_new - goal_point);
                        if (dist_from_goal < min_dist_from_goal)
                        {
                            min_dist_from_goal = dist_from_goal;
                            std::cout << "min dist from goal:" << dist_from_goal
                                      << std::endl;
                        }
                        if (dist_from_goal <= threshold)
                        {
                            stop_rrt = true;
                            goal_node = new_node;
                        }
                        break;
                    }
                }
            }
        }
        else
        {
            std::cout << "Cannot find Nearest Node" << std::endl;
        }
    }

    std::cout << "FINISHED RRT with size: " << rrt_size << std::endl;
    if (!stop_rrt)
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr whole_tree(
        new pcl::PointCloud<pcl::PointXYZ>);

    auto initial_path = get_path_between_two_nodes(RRT_graph, node_to_point,
                                                   first_node, goal_node);

    auto end_point = "end.xyz";
    cv::Point3f end_pointCV = cv::Point3f(
        initial_path.back().x, initial_path.back().y, initial_path.back().z);
    Auxilary::save_points_to_file({end_pointCV}, end_point);

    // refine path

    std::vector<pcl::PointXYZ> refined_path;
    int current = 0;
    refined_path.push_back(initial_path[current]);
    for (int i = static_cast<int>(initial_path.size()); i > current; i--)
    {
        if (is_valid_movement(cloud, initial_path[current], initial_path[i],
                              polygons))
        {
            refined_path.push_back(initial_path[i]);
            current = i;
            i = initial_path.size();
        }
    }

    // add goal point if last was dropped
    if (norm(goal_point - refined_path[0]) > 0.75)
        refined_path.push_back(goal_point);

    std::reverse(refined_path.begin(), refined_path.end());
    path_to_the_unknown = refined_path;

    std::cout << "\nrefined path - from " << initial_path.size() << " to "
              << refined_path.size() << " nodes!\n"
              << std::endl;

    for (lemon::ListDigraph::NodeIt it(RRT_graph); it != lemon::INVALID; ++it)
    {
        RRT_points.push_back(node_to_point[it]);
    }

    return true;
}

std::vector<pcl::PointXYZ> PathBuilder::build_path_to_exit(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ &start_point, const pcl::PointXYZ &exit_point,
    const pcl::PointXYZ &known_point1, const pcl::PointXYZ &known_point2,
    const pcl::PointXYZ &known_point3, std::vector<pcl::PointXYZ> &RRT_graph,
    const float threshold, const float jump_size, const int ring_point_amount,
    const float ring_size_scalar)
{
    std::vector<pcl::PointXYZ> path_to_exit;

    auto cluster_indices = get_clusters(cloud);
    auto convexhulls = get_convexhulls(cloud, cluster_indices);

    std::size_t tries = 30;
    while (path_to_exit.empty() && tries-- > 0)
    {
        std::cout << "Try number: " << 30 - tries << std::endl;
        std::cout << "NOT VALID" << std::endl;

        path_to_exit.clear();
        RRT_graph.clear();

        bool rrt_finished = get_navigation_points(
            cloud, start_point, known_point1, known_point2, known_point3,
            path_to_exit, RRT_graph, convexhulls, exit_point, threshold,
            jump_size, ring_point_amount, ring_size_scalar);

        if (!rrt_finished)
            continue;

        if (tries % 10 == 0)
        {
            cluster_indices = get_clusters(cloud);
            convexhulls = get_convexhulls(cloud, cluster_indices);
        }
    }

    if (debug)
    {
        for (const auto &point : path_to_exit)
        {
            std::cout << point.x << " " << point.y << " " << point.z << "->";
        }
    }

    return path_to_exit;
}
