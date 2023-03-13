#include "path_builder.hpp"

#include <geos/geom/GeometryFactory.h>
#include <geos/geom/PrecisionModel.h>
#include <lemon/core.h>
#include <lemon/list_graph.h>
#include <lemon/path.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>

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

#include "auxilary.hpp"

using namespace Auxilary;

PathBuilder::PathBuilder(float scale_factor) : scale_factor(scale_factor) {}

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
    std::size_t reached = 0;

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
    std::unique_ptr<geos::geom::PrecisionModel> pm(
        new geos::geom::PrecisionModel());
    auto geos_factory = geos::geom::GeometryFactory::create();

    std::vector<double> distances;
    std::vector<std::vector<std::size_t>> polygons_idxs;

    for (const auto& p : path) {
        std::vector<double> current_distances;

        std::transform(polygons.begin(), polygons.end(),
                       std::back_inserter(current_distances),
                       [&](const auto& poly) {
                           return poly->distance(
                               geos_factory->createPoint({p.x, p.y, p.z}));
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
        std::back_inserter(rates), [&](const auto& part) -> double {
            if (part.size() == 1) {
                return 0;
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
                return 0;
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

    lemon::ListDigraph RRT_graph;
    lemon::ListDigraph::NodeMap<pcl::PointXYZ> node_to_point(RRT_graph);
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
    std::unordered_map<pcl::PointXYZ, lemon::ListDigraph::Node,
                       decltype(hash_func), decltype(comp_func)>
        point_to_node(8, hash_func, comp_func);

    std::vector<Edge> v_edges;

    pcl::PointCloud<pcl::PointXYZ>::Ptr navigate_data(
        new pcl::PointCloud<pcl::PointXYZ>);

    navigate_data->push_back(navigate_starting_point);
    lemon::ListDigraph::Node first_node = RRT_graph.addNode();

    node_to_point[first_node] = navigate_starting_point;
    point_to_node[navigate_starting_point] = first_node;

    float jump = 0.3;
    // float jump = 0.2 * scale_factor;
    pcl::KdTreeFLANN<pcl::PointXYZ> navigate_tree;

    pcl::PointXYZ plane_mean = (known_point1 + known_point2 + known_point3) / 3;

    auto [cp, span_v1_gs, span_v2_gs, d] = get_plane_from_3_points(
        known_point1 - plane_mean, known_point2 - plane_mean,
        known_point3 - plane_mean);

    int rrt_size = 0;
    for (int i = 0; i < 4000; ++i) {
        // pcl::PointXYZ point_rand =
        //     (i % 3 == 0) ? get_random_point_on_plane(cp, d)
        //                  : get_random_point_on_plane(
        //                        navigate_starting_point,
        //                        *point_of_interest, span_v1_gs,
        //                        span_v2_gs, cp);
        pcl::PointXYZ point_rand =
            // (i % 3 != 0)
            // ? get_random_point_on_plane(cp, d)
            get_random_point_on_plane(span_v1_gs, span_v2_gs) + plane_mean;
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

                for (lemon::ListDigraph::NodeIt it(RRT_graph);
                     it != lemon::INVALID; ++it) {
                    if (node_to_point[it] == close_point) {
                        lemon::ListDigraph::Node new_node = RRT_graph.addNode();
                        node_to_point[new_node] = point_new;
                        point_to_node[point_new] = new_node;
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

    std::cout << "FINISHED RRT with size " << rrt_size << std::endl;

    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();

    if (lemon::countNodes(RRT_graph) < 50) return;

    std::random_device os_seed;
    const uint_least32_t seed = os_seed();

    std::mt19937 generator(seed);
    std::uniform_int_distribution<uint_least32_t> dist(
        1, lemon::countNodes(RRT_graph) - 2);
    lemon::ListDigraph::NodeIt n(RRT_graph);

    std::vector<int> v_closest_point(1);
    std::vector<float> v_dist_closest_point(1);

    const auto paths =
        get_all_paths_to_leaves(RRT_graph, node_to_point, first_node);

    // return;

    std::vector<
        std::pair<std::vector<double>, std::vector<std::vector<std::size_t>>>>
        polygon_distances;

    std::transform(paths.begin(), paths.end(),
                   std::back_inserter(polygon_distances),
                   [&](const auto& path) {
                       return get_top_k_polygon_distances(path, polygons);
                   });

    std::vector<double> rates;

    std::transform(polygon_distances.begin(), polygon_distances.end(),
                   std::back_inserter(rates), [](const auto& distances) {
                       return get_path_rate(std::get<0>(distances));
                   });

    const auto rate_indices = argsort(rates, true);

    const auto& max_distances = polygon_distances[rate_indices[0]];
    if (std::get<0>(max_distances).size() < how_long_valid_path) {
        path_to_the_unknown.clear();
        return;
    }

    const auto parts = split_distances(std::get<0>(max_distances));

    for (const auto& part : parts) {
        std::cout << "PART:" << std::endl;
        for (const auto& d : part) {
            std::cout << d << std::endl;
        }
    }

    std::transform(rate_indices.begin(), rate_indices.begin() + 10,
                   std::back_inserter(best_paths),
                   [&](const auto& ind) { return paths[ind]; });

    // const auto max_path =
    //     std::max_element(polygon_distances.begin(), polygon_distances.end(),
    //                      [](const auto& distances_a, const auto& distances_b)
    //                      {
    //                          return get_path_rate(std::get<0>(distances_a)) <
    //                                 get_path_rate(std::get<0>(distances_b));
    //                      });

    std::cout << "Rate of " << get_path_rate(std::get<0>(max_distances))
              << " is the max" << std::endl;

    std::cout << "Distances:" << std::endl;
    std::for_each(std::get<0>(max_distances).begin(),
                  std::get<0>(max_distances).end(),
                  [](const auto& ds) { std::cout << ds << std::endl; });

    path_to_the_unknown = paths[rate_indices[0]];

    std::size_t original_size = path_to_the_unknown.size();

    // path_to_the_unknown = {path_to_the_unknown.begin(),
    //                        path_to_the_unknown.begin() +
    //                            get_last_change(std::get<1>(max_distances))};

    path_to_the_unknown = {
        path_to_the_unknown.begin(),
        path_to_the_unknown.end() - parts[parts.size() - 1].size()};

    std::cout << "Path of size " << path_to_the_unknown.size() << " out of "
              << original_size << std::endl;

    for (lemon::ListDigraph::NodeIt it(RRT_graph); it != lemon::INVALID; ++it) {
        RRT_points.push_back(node_to_point[it]);
    }

    return;

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

    lemon::ListDigraph::Node get_random_node_rrt =
        point_of_interest == nullptr
            ? n
            : point_to_node[(*navigate_data)[v_closest_point[0]]];

    // BFS
    lemon::Bfs<lemon::ListDigraph> bfs(RRT_graph);
    bfs.init();
    bfs.addSource(first_node);
    bfs.start();

    if (bfs.reached(get_random_node_rrt)) {
        path_to_the_unknown.insert(path_to_the_unknown.begin(),
                                   node_to_point[get_random_node_rrt]);

        lemon::ListDigraph::Node prev = bfs.predNode(get_random_node_rrt);

        while (prev != lemon::INVALID) {
            path_to_the_unknown.insert(path_to_the_unknown.begin(),
                                       node_to_point[prev]);
            prev = bfs.predNode(prev);
        }
    }

    for (lemon::ListDigraph::NodeIt it(RRT_graph); it != lemon::INVALID; ++it) {
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

    // while (path_to_the_unknown.size() < how_long_valid_path && tries-- >
    // 0) {
    while (path_to_the_unknown.empty() && tries-- > 0) {
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
