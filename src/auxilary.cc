#include "auxilary.hpp"
#include <geos_c.h>

#include <geos.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/GeometryComponentFilter.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Polygon.h>
#include <initializer_list>

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "eigen_operations.hpp"
#include "pcl_operations.hpp"

using namespace PCLOperations;
using namespace EigenOperations;

namespace Auxilary
{
int radius_search(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                  const pcl::PointXYZ &search_point, float radius,
                  const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree)
{
    std::cout << radius << std::endl;
    std::vector<int> point_idx_radius_search;
    std::vector<float> point_radius_squared_distance;

    return kdtree.radiusSearch(search_point, radius, point_idx_radius_search,
                               point_radius_squared_distance);
}

std::vector<pcl::PointXYZ> get_points_on_line(const pcl::PointXYZ &start,
                                              const pcl::PointXYZ &end,
                                              float jump_distance)
{
    std::vector<pcl::PointXYZ> points_on_line;
    pcl::PointXYZ start_to_end = end - start;

    float total_travel_line = std::sqrt(start_to_end * start_to_end);

    pcl::PointXYZ hat_p = start_to_end / total_travel_line;
    for (float i = jump_distance; i < total_travel_line; i += jump_distance)
    {
        points_on_line.push_back(start + hat_p * i);
    }

    return points_on_line;
}

std::vector<std::unique_ptr<geos::geom::Geometry>>
get_convexhulls(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                const std::vector<pcl::PointIndices> &cluster_indices)
{
    std::vector<std::unique_ptr<geos::geom::Geometry>> convexhulls;
    auto geos_factory = geos::geom::GeometryFactory::create();

    for (const auto &i : cluster_indices)
    {
        std::vector<geos::geom::Coordinate> cluster_points;
        std::transform(i.indices.begin(), i.indices.end(),
                       std::back_inserter(cluster_points),
                       [&](const auto &p_index) -> geos::geom::Coordinate
                       {
                           const auto &p = (*cloud)[p_index];
                           return {p.x, p.y, p.z};
                       });

        auto multi_point =
            geos_factory->createMultiPoint(std::move(cluster_points));

        convexhulls.push_back(multi_point->convexHull());
    }

    return convexhulls;
}

bool check_polygon_intersection(
    const pcl::PointXYZ &start, const pcl::PointXYZ &end,
    const std::unique_ptr<geos::geom::Geometry> &polygon)
{
    auto geos_factory = geos::geom::GeometryFactory::create();
    const std::initializer_list<Coordinate> &coords_vec{
        {start.x, start.y, start.z}, {end.x, end.y, end.z}};

    auto coords = geos::geom::CoordinateSequence(coords_vec);

    auto line = geos_factory->createLineString(coords);

    return line->intersects(polygon.get());
}

bool is_valid_movement(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const pcl::PointXYZ &current_point, const pcl::PointXYZ &dest_point,
    const std::vector<std::unique_ptr<geos::geom::Geometry>> &polygons)
{
    return std::all_of(polygons.begin(), polygons.end(),
                       [&](const auto &polygon) {
                           return !check_polygon_intersection(
                               current_point, dest_point, polygon);
                       });
}

std::tuple<pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ, float>
get_plane_from_3_points(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2,
                        const pcl::PointXYZ &p3)
{
    pcl::PointXYZ span_v1 = p3 - p1;
    auto span_v1_gs = span_v1 / std::sqrt(span_v1 * span_v1);

    pcl::PointXYZ span_v2 = p2 - p1;
    auto span_v2_gs = span_v2 - (span_v2 * span_v1_gs) * span_v1_gs;
    span_v2_gs = span_v2_gs / std::sqrt(span_v2_gs * span_v2_gs);

    return {span_v1_gs, span_v1_gs, span_v2_gs, 3};
}

void save_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                   const std::vector<pcl::PointIndices> &cluster_indices)
{
    std::size_t j = 0;
    for (const auto &cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
            new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->is_dense = true;

        pcl::io::savePCDFileASCII("test_pcd" + std::to_string(j++) + ".pcd",
                                  *cloud_cluster);
    }
}

std::vector<pcl::PointIndices>
get_clusters(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int k,
             std::size_t minimum_cluster_size)
{
    cv::Mat points(cloud->size(), 1, CV_32FC3);
    cv::Mat labels;
    std::vector<cv::Point3f> centers;

    for (int i = 0; i < cloud->size(); ++i)
    {
        const auto &p = (*cloud)[i];
        points.at<cv::Point3f>(i) = cv::Point3f(p.x, p.y, p.z);
    }

    double compactness = cv::kmeans(
        points, k, labels, cv::TermCriteria(cv::TermCriteria::EPS, 10, 0.2), 10,
        cv::KMEANS_PP_CENTERS, centers);

    std::vector<pcl::PointIndices> cluster_indices(centers.size());

    for (int i = 0; i < cloud->size(); ++i)
    {
        cluster_indices[labels.at<int>(i)].indices.push_back(i);
    }

    cluster_indices.erase(
        std::remove_if(cluster_indices.begin(), cluster_indices.end(),
                       [&](const auto &indices) {
                           return indices.indices.size() < minimum_cluster_size;
                       }),
        cluster_indices.end());

    std::cout << "Found " << cluster_indices.size() << " clusters" << std::endl;

    return cluster_indices;
}

std::tuple<Eigen::MatrixXf, float, float>
get_distances(const std::vector<pcl::PointXYZ> &v1,
              const std::vector<pcl::PointXYZ> &v2)
{
    float min_val = std::numeric_limits<float>::max();
    float sum_non_zero = 0;
    float count_non_zero = 0;

    Eigen::MatrixXf dist_mat(v1.size(), v2.size());

    for (int i = 0; i < v1.size(); ++i)
    {
        for (int j = 0; j < v2.size(); ++j)
        {
            const auto diff = v2[j] - v1[i];
            const float dist = std::sqrt(diff * diff);
            if (dist != 0)
            {
                min_val = std::min(min_val, dist);
                sum_non_zero += dist;
                ++count_non_zero;
            }

            dist_mat(i, j) = dist;
        }
    }

    return {dist_mat, min_val,
            count_non_zero == 0 ? 0 : sum_non_zero / count_non_zero};
}

std::vector<std::size_t> get_random_indices(std::size_t vec_length,
                                            std::size_t amount)
{
    std::vector<std::size_t> indices;
    indices.reserve(amount);

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_int_distribution<std::size_t> dis(0, vec_length);

    std::generate_n(std::back_inserter(indices), amount,
                    [&] { return dis(gen); });

    return indices;
}

pcl::PointXYZ robust_median(const std::vector<pcl::PointXYZ> &points, int k)
{
    auto random_indices = get_random_indices(
        points.size(), static_cast<std::size_t>(3 * std::sqrt(points.size())));
    std::sort(random_indices.begin(), random_indices.end());
    random_indices.erase(
        std::unique(random_indices.begin(), random_indices.end()),
        random_indices.end());
    std::vector<pcl::PointXYZ> subset_points(random_indices.size());

    std::transform(random_indices.begin(), random_indices.end(),
                   subset_points.begin(),
                   [&](const auto &index) { return points[index]; });

    auto distances = std::get<0>(get_distances(subset_points, subset_points));

    sort_matrix_columns(distances);

    const auto dists_block = distances.block(
        0, 0,
        static_cast<int>(std::round(15 / (16 * k))) * random_indices.size(),
        distances.cols());

    std::vector<float> sums(dists_block.cols());

    for (Eigen::Index col = 0; col < dists_block.cols(); ++col)
    {
        sums[col] = dists_block.col(col).sum();
    }

    return points[random_indices[std::distance(
        sums.begin(), std::min_element(sums.begin(), sums.end()))]];
}

// NOTE: This function is not used at the moment, delete?
std::vector<pcl::PointXYZ> recursive_robust_median_clustering(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int k)
{
    auto points_eigen_alloc = cloud->points;
    std::vector<pcl::PointXYZ> points(
        std::make_move_iterator(points_eigen_alloc.begin()),
        std::make_move_iterator(points_eigen_alloc.end()));

    std::vector<pcl::PointXYZ> centers;

    Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1> Q;
    Q.setLinSpaced(static_cast<Eigen::Index>(points.size()) - 1, 0,
                   points.size() - 1);

    Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1> Q_temp;
    Q_temp.setLinSpaced(static_cast<Eigen::Index>(Q.size()) - 1, 0,
                        Q.size() - 1);
    pcl::PointXYZ center;

    while (Q.size() > 10 * static_cast<int>(std::log(points.size())))
    {
        Q_temp.setLinSpaced(static_cast<Eigen::Index>(Q.size()) - 1, 0,
                            Q.size() - 1);
        for (std::size_t i = 0;
             i < static_cast<std::size_t>(std::log(Q.size())); ++i)
        {
            auto points_subset = get_indices(points, get_indices(Q, Q_temp));
            auto center = robust_median(points_subset, k);
            std::vector<float> dist_from_center(points_subset.size());

            std::transform(points_subset.begin(), points_subset.end(),
                           dist_from_center.begin(),
                           [&](const auto &p)
                           {
                               const auto new_p = p - center;
                               return sqrt(new_p * new_p);
                           });

            auto idxs = argsort(dist_from_center);
            idxs = std::vector(idxs.begin(),
                               idxs.begin() + static_cast<std::size_t>(
                                                  dist_from_center.size() / 2));

            Q_temp = get_indices(Q_temp, idxs);
        }

        auto points_for_mean = get_indices(points, get_indices(Q, Q_temp));
        centers.push_back(
            std::accumulate(points_for_mean.begin(), points_for_mean.end(),
                            pcl::PointXYZ{0, 0, 0},
                            [](const pcl::PointXYZ &sum, const pcl::PointXYZ &p)
                            { return sum + p; }) /
            static_cast<float>(points_for_mean.size()));
        Q = get_all_indices_except(Q, Q_temp);
    }

    auto points_for_final_mean = get_indices(points, Q);
    centers.push_back(
        std::accumulate(points_for_final_mean.begin(),
                        points_for_final_mean.end(), pcl::PointXYZ{0, 0, 0},
                        [](const pcl::PointXYZ &sum, const pcl::PointXYZ &p)
                        { return sum + p; }) /
        static_cast<float>(points_for_final_mean.size()));

    return centers;
}
std::vector<float> getRandomPointFrom3DRing(std::vector<float> &center, float R,
                                            float r)
{
    float norm_of_point = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0, R - r);
    std::uniform_real_distribution<float> dis2(-1, 1);
    pcl::PointXYZ rand_vec = pcl::PointXYZ(dis2(gen), dis2(gen), dis2(gen));
    float norm_of_rand_vec = 0;
    float v = dis(gen) + r;
    std::vector<float> randVec{rand_vec.x, rand_vec.y};
    for (int i = 0; i < 2; i++)
    {
        norm_of_rand_vec += (randVec[i] * randVec[i]);
    }

    norm_of_rand_vec = std::sqrt(norm_of_rand_vec);
    for (int i = 0; i < 2; i++)
    {
        randVec[i] = randVec[i] / norm_of_rand_vec * v + center[i];
    }
    return {randVec[0], randVec[1]};
}

pcl::PointXYZ get_random_point_on_plane(const pcl::PointXYZ &start,
                                        const pcl::PointXYZ &point_of_interest,
                                        const pcl::PointXYZ &u1,
                                        const pcl::PointXYZ &u2,
                                        const pcl::PointXYZ &cp)
{
    auto start_proj = u1 * (u1 * start) + u2 * (u2 * start);
    auto end_proj =
        u1 * (u1 * point_of_interest) + u2 * (u2 * point_of_interest);

    auto normal_to_plane =
        cross_product(cp / std::sqrt(cp * cp), start_proj - end_proj);

    auto diff = end_proj - start_proj;

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<float> dis_1(0,
                                                2 * std::sqrt((diff * diff)));
    std::uniform_real_distribution<float> dis_2(0, std::sqrt((diff * diff)));

    return dis_1(gen) * (end_proj - start_proj) + dis_2(gen) * normal_to_plane;
}

pcl::PointXYZ get_random_point_on_plane(const pcl::PointXYZ &span_v1,
                                        const pcl::PointXYZ &span_v2,
                                        std::vector<float> &center, float R,
                                        float r)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    // std::normal_distribution<float> dis(0, std::sqrt(50));
    std::uniform_real_distribution<float> dis(-30, 30);
    // get the x and y random distrubtions from getRandomPointFrom3DRing
    // function and use it instead of dis(gen)
    std::vector<float> vec = getRandomPointFrom3DRing(center, R, r);
    // float rand_y = (getRandomPointFrom3DRing(center, R, r))[1];

    return vec[0] * span_v1 + vec[1] * span_v2;
}

pcl::PointXYZ get_random_point_on_plane(const pcl::PointXYZ &cp, float d)
{
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<float> dis(-30, 30);
    auto x = dis(gen);
    auto y = dis(gen);
    return {x, y, (-d - cp.x * x - cp.y * y) / (cp.z == 0 ? 0.001f : cp.z)};
}

std::vector<pcl::PointXYZ>
load_path_from_file(const std::filesystem::path &location_file_path)
{
    std::vector<pcl::PointXYZ> path_to_unknown;

    std::ifstream fin(location_file_path);

    std::vector<float> values;
    if (fin.good())
    {
        float value = 0;
        while (fin >> value)
        {
            values.push_back(value);
            if (values.size() == 3)
            {
                path_to_unknown.emplace_back(values[0], values[1], values[2]);
                values.clear();
            }
        }
    }

    return path_to_unknown;
}

void save_points_to_file(const std::vector<pcl::PointXYZ> &points,
                         const std::filesystem::path &location_file_path)
{
    std::ofstream file_of_path(location_file_path);

    for (const auto &point : points)
    {
        file_of_path << point.x << " " << point.y << " " << point.z
                     << std::endl;
    }

    file_of_path.close();
}

void save_points_to_file(const std::vector<Eigen::Matrix<double, 3, 1>> &points,
                         const std::filesystem::path &location_file_path)
{
    std::ofstream file_of_path(location_file_path);

    for (const auto &point : points)
    {
        file_of_path << point.x() << " " << point.y() << " " << point.z()
                     << std::endl;
    }

    file_of_path.close();
}

void save_points_to_file(const std::vector<cv::Point3f> &points,
                         const std::filesystem::path &location_file_path)
{
    std::ofstream file_of_path(location_file_path);

    for (const auto &point : points)
    {
        file_of_path << point.x << " " << point.y << " " << point.z
                     << std::endl;
    }

    file_of_path.close();
}

} // namespace Auxilary
