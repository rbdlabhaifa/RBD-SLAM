#include "path_builder.hpp"

#include <geos/geom/GeometryFactory.h>
#include <lemon/core.h>
#include <lemon/list_graph.h>
#include <lemon/path.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <array>
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
#include <pcl/impl/point_types.hpp>
#include "auxilary.hpp"
#include "eigen_operations.hpp"
#include "pcl_operations.hpp"
#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>

#include <opencv2/core.hpp>
using namespace Auxilary;
using namespace PCLOperations;
using namespace EigenOperations;

std::size_t PathBuilder::get_last_change(
    const std::vector<std::vector<std::size_t>>& polygon_idxs) {
    auto p_idxs = polygon_idxs;

    std::for_each(p_idxs.begin(), p_idxs.end(),
                  [](auto& idxs) { std::sort(idxs.begin(), idxs.end()); });

    std::size_t last_change = 1;

    while (last_change < p_idxs.size() &&
           p_idxs[last_change] != p_idxs[last_change - 1]) {
        ++last_change;
    }

    return last_change - 1;
}

std::vector<std::vector<pcl::PointXYZ>> PathBuilder::get_all_paths_to_leaves(
    const lemon::ListDigraph& graph,
    const lemon::ListDigraph::NodeMap<pcl::PointXYZ>& node_map,
    const lemon::ListDigraph::Node& start_node) {
    std::vector<std::vector<pcl::PointXYZ>> paths;

    std::size_t amount = 0;

    lemon::Dfs<lemon::ListDigraph> dfs(graph);
    dfs.init();
    dfs.addSource(start_node);
    dfs.start();

    std::size_t leaves = 0;

    for (lemon::ListDigraph::NodeIt it(graph); it != lemon::INVALID; ++it) {
        if (lemon::countOutArcs(graph, it) != 0) {
            continue;
        }

        ++leaves;

        std::vector<pcl::PointXYZ> current_path;
        current_path.insert(current_path.begin(), node_map[it]);

        auto prev = dfs.predNode(it);

        while (prev != lemon::INVALID) {
            current_path.insert(current_path.begin(), node_map[prev]);
            prev = dfs.predNode(prev);
        }

        paths.push_back(current_path);
    }

    std::cout << leaves << " LEAVES from " << lemon::countNodes(graph)
              << std::endl;

    return paths;
}

std::pair<std::vector<double>, std::vector<std::vector<std::size_t>>>
PathBuilder::get_top_k_polygon_distances(
    const std::vector<pcl::PointXYZ>& path,
    const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons, int k) {
    auto geos_factory = geos::geom::GeometryFactory::create();

    std::vector<double> distances;
    std::vector<std::vector<std::size_t>> polygons_idxs;

    for (const auto& p : path) {
        std::vector<double> current_distances;
        geos::geom::Coordinate coard(p.x, p.y, p.z);
        std::transform(polygons.begin(), polygons.end(),
                        std::back_inserter(current_distances),
                        [&](const auto& poly)
            {
                return poly->distance(geos_factory->createPoint(coard).get());
            });

        // TODO: maybe we dont need to sort all distances
        auto current_idxs = argsort(current_distances);

        for (std::size_t i = 0; i < k; ++i) {
            current_distances[i] = current_distances[current_idxs[i]];
        }

        distances.emplace_back(std::accumulate(current_distances.begin(),
                                               current_distances.begin() + k,
                                               0.) /
                               k);
        polygons_idxs.emplace_back(current_idxs.begin(),
                                   current_idxs.begin() + k);
    }

    return {distances, polygons_idxs};
}

// Implementation taken from
// https://github.com/numpy/numpy-financial/blob/main/numpy_financial/_financial.py
Eigen::VectorXd PathBuilder::g_div_gp(const Eigen::VectorXd& fv,
                                      const Eigen::VectorXd& pv,
                                      const Eigen::VectorXd& rn, int w,
                                      int nper, int pmt) {
    const auto& fv_arr = fv.array();
    const auto& pv_arr = pv.array();
    const auto& rn_arr = rn.array();

    Eigen::VectorXd g;
    Eigen::VectorXd gp;

    // TODO: This check may be improved
    if (rn_arr.size() == 1) {
        auto t1 = std::pow(rn_arr[0] + 1, nper);
        auto t2 = std::pow(rn_arr[0] + 1, nper - 1);

        g = fv_arr + t1 * pv_arr +
            pmt * (t1 - 1) * (rn_arr[0] * w + 1) / rn_arr[0];
        gp = nper * t2 * pv_arr -
             pmt * (t1 - 1) * (rn_arr[0] * w + 1) / std::pow(rn_arr[0], 2) +
             nper * pmt * t2 * (rn_arr[0] * w + 1) / rn_arr[0] +
             pmt * (t1 - 1) * w / rn_arr[0];
    } else {
        auto t1 = (rn_arr + 1).pow(nper);
        auto t2 = (rn_arr + 1).pow(nper - 1);

        g = fv_arr + t1 * pv_arr + pmt * (t1 - 1) * (rn_arr * w + 1) / rn_arr;
        gp = nper * t2 * pv_arr -
             pmt * (t1 - 1) * (rn_arr * w + 1) / rn_arr.pow(2) +
             nper * pmt * t2 * (rn_arr * w + 1) / rn_arr +
             pmt * (t1 - 1) * w / rn_arr;
    }

    return g.array() / gp.array();
}

std::vector<std::vector<double>> PathBuilder::split_distances(
    const std::vector<double>& distances) {
    std::vector<std::vector<double>> distances_parts;

    int sign = 0;
    std::size_t start = 0;
    std::size_t end = 0;
    bool get_part = false;

    for (std::size_t i = 0; i < distances.size() - 1; ++i) {
        if (distances[i] <= distances[i + 1]) {
            get_part = sign == -1;

            if (sign == 0) {
                sign = 1;
                start = i;
            }
        } else {
            get_part = sign == 1;

            if (sign == 0) {
                sign = -1;
                start = i;
            }
        }

        if (get_part) {
            sign = 0;
            distances_parts.emplace_back(distances.begin() + start,
                                         distances.begin() + i + 1);
            start = i;
        }

        get_part = false;
    }

    distances_parts.emplace_back(distances.begin() + start, distances.end());

    return distances_parts;
}

double PathBuilder::get_path_rate(const std::vector<double>& distances) {
    if (distances.size() == 1) {
        return 0;
    }

    const double guess = 0.1;
    const int nper = 1;
    const int pmt = 0;
    const int maxiter = 100;
    const double tol = 1e-6;

    const auto distances_parts = split_distances(distances);
    std::vector<double> rates;

    std::transform(
        distances_parts.begin(), distances_parts.end(),
        std::back_inserter(rates), [&](const auto& part) {
            if (part.size() == 1) {
                return 0.;
            }

            auto dist_copy = part;
            Eigen::VectorXd pv = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
                dist_copy.data(), dist_copy.size());
            Eigen::VectorXd fv(pv);

            remove_row(pv, pv.size() - 1);
            remove_row(fv, 0);

            pv = -pv;
            Eigen::VectorXd rn(1);
            rn << guess;

            Eigen::Array<bool, Eigen::Dynamic, 1> close(1);
            close << false;

            const int w = 0;

            for (int i = 0; i < maxiter && !close.all(); ++i) {
                if (rn.size() == 1) {
                    Eigen::ArrayXd rnp1 =
                        -g_div_gp(fv, pv, rn, w, nper, pmt).array() + rn[0];

                    Eigen::ArrayXd diff = rnp1 - rn[0];
                    diff = diff.abs();
                    rn = rnp1;
                    close = diff < tol;
                } else {
                    Eigen::ArrayXd rnp1 =
                        -g_div_gp(fv, pv, rn, w, nper, pmt).array() +
                        rn.array();

                    Eigen::ArrayXd diff = rnp1 - rn.array();

                    diff = diff.abs();
                    rn = rnp1;
                    close = diff < tol;
                }
            }

            if (!close.all()) {
                return 0.;
            }

            return rn.mean();
        });

    if (!rates.empty()) {
        const auto& max_rate = std::max_element(rates.begin(), rates.end());

        if (std::distance(rates.begin(), max_rate) == rates.size() - 1) {
            return *max_rate;
        }
    }

    return 0;
}

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
    std::cout << "min_val is " << min_val << " mean_val is " << mean_val << std::endl;
    float* R_s_data = R_s.data();
    Eigen::VectorXf R_s_eigen =
        Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(R_s_data, R_s.size());
    for (int i = 0; i < cloud->size(); ++i) {
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

void saveTree(const std::vector<std::vector<pcl::PointXYZ>>& paths){
    std::cout << "rrt path points:" << std::endl;
    std::string filename = "tree.csv";
    std::ofstream file(filename);
    if(!file.is_open()) { 
    	std::cerr << "Error opening file: " << filename << std::endl;
    }
    for(size_t i = 0; i< paths.size() ; ++i){
    	const auto& cloud = paths[i];
    	for ( const auto& point : cloud){
    		file << point.x << " " << point.y << " " << point.z << std::endl;
    	}
    }
    file.close();
    std::cout<< " point cloud saved to : " << filename<< std::endl;
}

void savePaths(const std::vector<std::vector<pcl::PointXYZ>>& paths){
    std::string filename1 = "paths.csv";
    std::ofstream file1(filename1);
    if(!file1.is_open()) { 
    	std::cerr << "Error opening file: " << filename1 << std::endl;
    }
    for(size_t i = 0; i< paths.size() ; ++i){
    	const auto& path = paths[i];
    	for ( const auto& point : path){
    		file1 << point.x << " " << point.y << " " << point.z << " " ;
    	}
        file1 << std::endl;
    }
    file1.close();
    std::cout<< " paths saved to : " << filename1 << std::endl;
}

std::vector<pcl::PointXYZ> PathBuilder::find_best_path(
    const lemon::ListDigraph& graph,
    const lemon::ListDigraph::NodeMap<pcl::PointXYZ>& node_map,
    const lemon::ListDigraph::Node& start_node,
    const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons) {
    const auto paths = get_all_paths_to_leaves(graph, node_map, start_node);
    saveTree(paths);
    savePaths(paths);

    std::vector<
        std::pair<std::vector<double>, std::vector<std::vector<std::size_t>>>>
        polygon_distances;

    std::transform(paths.begin(), paths.end(),
                   std::back_inserter(polygon_distances),
                   [&](const auto& path) {
                       return get_top_k_polygon_distances(path, polygons);
                   });

    std::vector<double> rates;
    std::cout << "Starting distances computation" << std::endl;
    std::transform(polygon_distances.begin(), polygon_distances.end(),
                   std::back_inserter(rates), [](const auto& distances) {
                       return get_path_rate(std::get<0>(distances));
                   });
    
    std::cout << "Ended  distances computation" << std::endl;

    const auto rate_indices = argsort(rates, true);
    
    std::cout << "Sorted rates" << std::endl;
    int counter = 0;
    int idx = 0;
    auto& max_distances = polygon_distances[rate_indices[idx]];
    while(1){
        max_distances = polygon_distances[rate_indices[idx]];
        if (std::get<0>(max_distances).size() < how_long_valid_path)
        {
            idx++;
        }else {
            break;
        }
        if (counter >= 60) {
            std::cout << "bad!!!!!!" << std::endl;
            return {};
        }
    }
    
    std::cout << "splitted distances started" << std::endl;
    const auto parts = split_distances(std::get<0>(max_distances));
    std::cout << "splitted distances ended" << std::endl;

    for (const auto& part : parts) {
        std::cout << "PART:" << std::endl;
        for (const auto& d : part) {
            std::cout << d << std::endl;
        }
    }

    std::transform(rate_indices.begin(), rate_indices.begin() + 10,
                   std::back_inserter(best_paths),
                   [&](const auto& ind) { return paths[ind]; });

    std::cout << "Rate of " << get_path_rate(std::get<0>(max_distances))
              << " is the max" << std::endl;

    std::cout << "Distances:" << std::endl;
    std::for_each(std::get<0>(max_distances).begin(),
                  std::get<0>(max_distances).end(),
                  [](const auto& ds) { std::cout << ds << std::endl; });

    auto path_to_the_unknown = paths[rate_indices[idx]];

    std::size_t original_size = path_to_the_unknown.size();

    // TODO: maybe we need something else than all parts before last
    path_to_the_unknown = {
        path_to_the_unknown.begin(),
        path_to_the_unknown.end() - parts[parts.size() - 1].size()};

    std::cout << "Path of size " << path_to_the_unknown.size() << " out of "
              << original_size << std::endl;

    return path_to_the_unknown;
}

// TODO: Make this function better, use utility functions
void PathBuilder::get_navigation_points(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ& navigate_starting_point,
    const pcl::PointXYZ& known_point1, const pcl::PointXYZ& known_point2,
    const pcl::PointXYZ& known_point3,
    std::vector<pcl::PointXYZ>& path_to_the_unknown,
    std::vector<pcl::PointXYZ>& RRT_points,
    const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    std::cout << "vector RRT size: " << path_to_the_unknown.size() << std::endl;
    std::cout << "rrt ponts tree:" << std::endl;
    for( const auto& node : path_to_the_unknown){
    	std::cout << "we are here" << std::endl;
    	std::cout << node.x << " " << node.y << " " << node.z << " " << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr RRT_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    lemon::ListDigraph RRT_graph;
    lemon::ListDigraph::NodeMap<pcl::PointXYZ> node_to_point(RRT_graph);

    pcl::PointCloud<pcl::PointXYZ>::Ptr navigate_data(
        new pcl::PointCloud<pcl::PointXYZ>);

    navigate_data->push_back(navigate_starting_point);
    lemon::ListDigraph::Node first_node = RRT_graph.addNode();
    auto map_filename = "start.xyz";
    cv::Point3f navigateCv = cv::Point3f(navigate_starting_point.x,navigate_starting_point.y,navigate_starting_point.z);
    Auxilary::save_points_to_file({navigateCv}, map_filename);

    node_to_point[first_node] = navigate_starting_point;

    float jump = 0.3;

    pcl::KdTreeFLANN<pcl::PointXYZ> navigate_tree;
    pcl::PointXYZ plane_mean = (known_point1 + known_point2 + known_point3) / 3;

    auto [cp, span_v1_gs, span_v2_gs, d] = get_plane_from_3_points(
        known_point1 - plane_mean, known_point2 - plane_mean,
        known_point3 - plane_mean);
	// shershor for the A's to calculate the center
    std::cout << "starting shershor" << std::endl;
    //Eigen::Matrix2d A(2,3);
    cv::Mat A;
    A = cv::Mat::zeros(2, 3, CV_32F);
    A.at<float>(0,0) = span_v1_gs.x; 
    A.at<float>(0,1) = span_v1_gs.y;
    A.at<float>(0,2) = span_v1_gs.z;
    A.at<float>(1,0) = span_v2_gs.x;
    A.at<float>(1,1) = span_v2_gs.y;
    A.at<float>(1,2) = span_v2_gs.z;
    std::cout << "We are building A..." << std::endl;
  
    cv::Mat X = cv::Mat::zeros(1, 3, CV_32F);
    cv::Mat Ainv = cv::Mat::zeros(3, 2, CV_32F);
    
    std::cout << "We did the inverse matrix A !!!" << std::endl;
    invert(A, Ainv, cv::DECOMP_SVD);
    
    X.at<float>(0,0) = navigate_starting_point.x;
    X.at<float>(0,1) = navigate_starting_point.y;
    X.at<float>(0,2) = navigate_starting_point.z;
    cv::Mat Center = cv::Mat::zeros(1, 2, CV_32F);
    
    std::cout << "the A inverse is: "<< std::endl;
    Center = X * Ainv;
    std::cout << "The center is: " << std::endl;
    int rrt_size = 0;
    std::vector<float> Cntr{Center.at<float>(0,0), Center.at<float>(0,1), Center.at<float>(0,2)};
    for (int i = 0; i < rrt_graph_max_size; ++i) {
        float R = ((i / 300) + 0.5) * 1.5; 
	    float r = (i / 300 ) * 1.5;
        pcl::PointXYZ point_rand =
            get_random_point_on_plane(span_v1_gs, span_v2_gs,Cntr, R, r) + plane_mean;

        navigate_tree.setInputCloud(navigate_data);

        std::vector<int> v_closest_point(1);
        std::vector<float> v_dist_closest_point(1);

        if (navigate_tree.nearestKSearch(point_rand, 1, v_closest_point,
                                         v_dist_closest_point) > 0) {
            const pcl::PointXYZ& close_point =
                (*navigate_data)[v_closest_point[0]];

            pcl::PointXYZ point_new =
                close_point + jump * (point_rand - close_point) /
                                  (std::sqrt((point_rand - close_point) *
                                             (point_rand - close_point)));

            if (is_valid_movement(cloud, close_point, point_new, polygons)) {
                for (lemon::ListDigraph::NodeIt it(RRT_graph);
                     it != lemon::INVALID; ++it) {
                    if (node_to_point[it] == close_point) {
                        navigate_data->push_back(point_new);
                        lemon::ListDigraph::Node new_node = RRT_graph.addNode();
                        node_to_point[new_node] = point_new;
                        RRT_graph.addArc(it, new_node);
                        ++rrt_size;
                        break;
                    }
                }
            }
        } else {
            std::cout << "Cannot find Nearest Node" << std::endl;
        }
    }

    std::cout << "FINISHED RRT with size: " << rrt_size << std::endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr whole_tree(
        new pcl::PointCloud<pcl::PointXYZ>);
    
    for (int i = 0; i < rrt_graph_max_size; ++i) {
    	
    }
    if (lemon::countNodes(RRT_graph) < 50) return;

    path_to_the_unknown =
        find_best_path(RRT_graph, node_to_point, first_node, polygons);
        
    auto end_point = "end.xyz";
    cv::Point3f end_pointCV = cv::Point3f(path_to_the_unknown.back().x,path_to_the_unknown.back().y,path_to_the_unknown.back().z);
    Auxilary::save_points_to_file({end_pointCV}, end_point);

    for (lemon::ListDigraph::NodeIt it(RRT_graph); it != lemon::INVALID; ++it) {
        RRT_points.push_back(node_to_point[it]);
    }
}

std::vector<pcl::PointXYZ> PathBuilder::operator()(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
    const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3,
    std::vector<pcl::PointXYZ>& RRT_points) {
    std::vector<pcl::PointXYZ> path_to_the_unknown;

    auto cluster_indices = get_clusters(cloud);
    auto convexhulls = get_convexhulls(cloud, cluster_indices);

    std::size_t tries = 30;
    while (path_to_the_unknown.empty() && tries-- > 0) {
        std::cout << "NOT VALID" << std::endl;

        path_to_the_unknown.clear();
        RRT_points.clear();

        get_navigation_points(cloud, start_point, known_point1, known_point2,
                              known_point3, path_to_the_unknown, RRT_points,
                              convexhulls);

        if (tries % 10 == 0) {
            cluster_indices = get_clusters(cloud);
            convexhulls = get_convexhulls(cloud, cluster_indices);
        }
    }

    if (debug) {
        for (const auto& point : path_to_the_unknown) {
            std::cout << point.x << " " << point.y << " " << point.z << "->";
        }
    }

    return path_to_the_unknown;
}

void PathBuilder::operator()(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                             const pcl::PointXYZ& start_point,
                             const pcl::PointXYZ& known_point1,
                             const pcl::PointXYZ& known_point2,
                             const pcl::PointXYZ& known_point3,
                             std::vector<pcl::PointXYZ>& path_to_the_unknown,
                             std::vector<pcl::PointXYZ>& RRT_points) {
    path_to_the_unknown = operator()(cloud, start_point, known_point1,
                                     known_point2, known_point3, RRT_points);
}

void PathBuilder::operator()(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ& start_point, const pcl::PointXYZ& known_point1,
    const pcl::PointXYZ& known_point2, const pcl::PointXYZ& known_point3,
    const std::filesystem::path& location_file_path_to_the_unknown,
    std::vector<pcl::PointXYZ>& RRT_points) {
    std::vector<pcl::PointXYZ> path_to_the_unknown = operator()(
        cloud, start_point, known_point1, known_point2, known_point3,
        RRT_points);
    save_points_to_file(path_to_the_unknown, location_file_path_to_the_unknown);
}



