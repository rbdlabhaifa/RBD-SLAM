#include "auxilary.hpp"

#include <Eigen/src/Core/util/Meta.h>
#include <geom/GeometryComponentFilter.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/PrecisionModel.h>
#include <geos/triangulate/polygon/ConstrainedDelaunayTriangulator.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ml/kmeans.h>
#include <pcl/segmentation/extract_clusters.h>
#include <triangulate/tri/Tri.h>
#include <triangulate/tri/TriList.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Auxilary {
    int radius_search(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                      const pcl::PointXYZ& search_point, float radius,
                      const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree) {
        std::vector<int> point_idx_radius_search;
        std::vector<float> point_radius_squared_distance;

        return kdtree.radiusSearch(search_point, radius,
                                   point_idx_radius_search,
                                   point_radius_squared_distance);
    }

    pcl::PointXYZ vec_to_pcl(std::vector<double> vec) {
        if (vec.size() != 3) return {0, 0, 0};

        return {static_cast<float>(vec[0]), static_cast<float>(vec[1]),
                static_cast<float>(vec[2])};
    }

    pcl::PointXYZ operator-(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
        return {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z};
    }

    pcl::PointXYZ operator/(const pcl::PointXYZ& p, float d) {
        if (d == 0) return p;

        return {p.x / d, p.y / d, p.z / d};
    }

    float norm(const pcl::PointXYZ& p) {
        return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    float operator*(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
        return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
    }

    pcl::PointXYZ operator*(const pcl::PointXYZ& p, float a) {
        return {p.x * a, p.y * a, p.z * a};
    }
    pcl::PointXYZ operator*(float a, const pcl::PointXYZ& p) {
        return {p.x * a, p.y * a, p.z * a};
    }

    pcl::PointXYZ operator+(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
        return {p1.x + p2.x, p1.y + p2.y, p1.z + p2.z};
    }

    bool operator==(const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
    }

    bool operator==(const pcl::PointXYZ lhs, const pcl::PointXYZ rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
    }

    template <typename T>
    std::vector<std::size_t> argsort(const std::vector<T>& vec) {
        std::vector<size_t> indices(vec.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(
            indices.begin(), indices.end(),
            [&vec](auto left, auto right) { return vec[left] < vec[right]; });

        return indices;
    }

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> get_indices(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& vec,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices) {
        Eigen::Matrix<T, Eigen::Dynamic, 1> block_mat(indices.size());

        for (Eigen::Index i = 0; i < indices.rows(); ++i) {
            block_mat(i) = vec(indices[i]);
        }

        return block_mat;
    }

    template <typename T>
    void remove_row(Eigen::Matrix<T, Eigen::Dynamic, 1>& matrix,
                    Eigen::Index rowToRemove) {
        Eigen::Index numRows = matrix.rows() - 1;
        Eigen::Index numCols = matrix.cols();

        if (rowToRemove < numRows) {
            matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) =
                matrix.block(rowToRemove + 1, 0, numRows - rowToRemove,
                             numCols);
        }

        matrix.conservativeResize(numRows, numCols);
    }

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> get_all_indices_except(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& vec,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices) {
        auto block_vec = vec;

        for (Eigen::Index i = 0; i < indices.size(); ++i) {
            remove_row(block_vec, indices[i]);
        }

        return block_vec;
    }

    template <typename T>
    std::vector<T> get_indices(
        const std::vector<T>& vec,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices) {
        std::vector<T> v_block(indices.size());

        for (Eigen::Index i = 0; i < indices.rows(); ++i) {
            v_block[i] = vec[indices[i]];
        }

        return v_block;
    }

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> get_indices(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& vec,
        const std::vector<std::size_t>& indices) {
        Eigen::Matrix<T, Eigen::Dynamic, 1> block_mat(indices.size());

        for (Eigen::Index i = 0; i < indices.size(); ++i) {
            block_mat(i) = vec(indices[i]);
        }

        return block_mat;
    }

    std::vector<pcl::PointXYZ> get_points_on_line(const pcl::PointXYZ& start,
                                                  const pcl::PointXYZ& end,
                                                  float jump_distance) {
        std::vector<pcl::PointXYZ> points_on_line;
        pcl::PointXYZ start_to_end = end - start;

        float total_travel_line = std::sqrt(start_to_end * start_to_end);

        pcl::PointXYZ hat_p = start_to_end / total_travel_line;
        for (float i = jump_distance; i < total_travel_line;
             i += jump_distance) {
            points_on_line.push_back(start + hat_p * i);
        }

        return points_on_line;
    }

    std::vector<std::unique_ptr<geos::geom::Geometry>> get_polygons(
        const std::vector<std::vector<pcl::PointXYZ>>& points) {
        std::vector<std::unique_ptr<geos::geom::Geometry>> polygons;
        std::unique_ptr<geos::geom::PrecisionModel> pm(
            new geos::geom::PrecisionModel());
        auto geos_factory = geos::geom::GeometryFactory::create(pm.get(), -1);

        for (const auto& points_v : points) {
            std::vector<geos::geom::Coordinate> coords_vec;

            std::transform(points_v.begin(), points_v.end(),
                           std::back_inserter(coords_vec),
                           [&](const auto& p) -> geos::geom::Coordinate {
                               return {p.x, p.y, p.z};
                           });

            auto coords = geos_factory->getCoordinateSequenceFactory()->create(
                std::move(coords_vec), 3);

            auto ring = geos_factory->createLinearRing();
            ring->setPoints(coords.get());
            // auto ring = geos_factory->createLinearRing(std::move(coords));

            auto poly = ring->convexHull();
            if (poly->isValid()) {
                polygons.push_back(std::move(poly));
            } else {
                std::cout << "NOT VALID POLYGON" << std::endl;
            }
        }

        return polygons;
    }

    std::vector<std::unique_ptr<geos::geom::Geometry>> get_convexhulls(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const std::vector<pcl::PointIndices>& cluster_indices) {
        std::vector<std::unique_ptr<geos::geom::Geometry>> convexhulls;
        std::unique_ptr<geos::geom::PrecisionModel> pm(
            new geos::geom::PrecisionModel());
        auto geos_factory = geos::geom::GeometryFactory::create(pm.get(), -1);

        for (const auto& i : cluster_indices) {
            std::vector<geos::geom::Coordinate> cluster_points;
            std::transform(i.indices.begin(), i.indices.end(),
                           std::back_inserter(cluster_points),
                           [&](const auto& p_index) -> geos::geom::Coordinate {
                               const auto& p = (*cloud)[p_index];
                               return {p.x, p.y, p.z};
                           });

            auto multi_point =
                geos_factory->createMultiPoint(std::move(cluster_points));

            convexhulls.push_back(multi_point->convexHull());
        }

        return convexhulls;
    }

    bool check_polygon_intersection(
        const pcl::PointXYZ& start, const pcl::PointXYZ& end,
        const std::unique_ptr<geos::geom::Geometry>& polygon) {
        std::unique_ptr<geos::geom::PrecisionModel> pm(
            new geos::geom::PrecisionModel());
        auto geos_factory = geos::geom::GeometryFactory::create(pm.get(), -1);
        std::vector<geos::geom::Coordinate> coords_vec{
            {start.x, start.y, start.z}, {end.x, end.y, end.z}};

        auto coords = geos_factory->getCoordinateSequenceFactory()->create(
            std::move(coords_vec), 3);

        auto line = geos_factory->createLineString(std::move(coords));

        // auto ring = geos_factory->createLinearRing();

        // std::vector<geos::geom::Coordinate> coords_vec2{
        //     {0.786114, -0.13823, -0.0470239},
        //     {0.697524, -0.175287, -0.160328},
        //     {0.765082, -0.135979, -0.0233465},
        //     {0.72338, -0.278474, -0.270879},
        //     {0.937708, -0.254209, -0.146081},
        //     {0.775675, -0.26956, 0.0602451},
        //     {0.956903, -0.165622, -0.0245822},
        //     {0.652139, -0.347795, -0.0178738},
        //     {0.661282, -0.326339, 0.0210348},
        //     {0.653851, -0.327743, -0.0690501},
        //     {0.651767, -0.325676, -0.0659045},
        //     {0.674744, -0.252031, 0.0450594},
        //     {0.705338, -0.164088, 0.0494387},
        //     {0.688923, -0.23809, 0.0698805},
        //     {0.758879, -0.22439, 0.0836618},
        //     {0.693623, -0.142233, 0.0171078},
        //     {0.793621, -0.152151, 0.01},
        //     {0.729003, -0.344593, -0.0400155},
        //     {0.750085, -0.326042, 0.00642205},
        //     {0.66272, -0.354988, -0.0275128},
        //     {0.757392, -0.33751, -0.0738401}};

        // auto coords_seq =
        // geos_factory->getCoordinateSequenceFactory()->create(
        //     std::move(coords_vec2), 3);

        // ring->setPoints(coords_seq.get());

        // auto poly = geos_factory->createPolygon(std::move(ring));
        return line->intersects(polygon.get());

        // return line->intersects(polygon.get());
        // return polygon->intersects(line.get());
        // return polygon->intersects(line.get());
    }

    bool check_convexhull_intersection(const pcl::PointXYZ& p,
                                       const ConvexHullEquations& convexhull,
                                       double tolerance) {
        return std::all_of(
            convexhull.facets_planes.begin(), convexhull.facets_planes.end(),
            [&](const auto& facet) {
                return facet.normal * p + facet.offset <= tolerance;
            });
    }

    bool check_convexhull_intersection(const pcl::PointXYZ& start,
                                       const pcl::PointXYZ& end,
                                       const ConvexHullEquations& convexhull) {
        const auto start_end = end - start;
        float tfirst = 0;
        float tlast = 0;

        for (const auto& facet : convexhull.facets_planes) {
            const auto denom = facet.normal * start_end;
            const auto dist = facet.offset - facet.normal * start;

            const auto t = dist / denom;
            if (denom > 0) {
            }

            return std::count_if(
                       convexhull.facets_planes.begin(),
                       convexhull.facets_planes.end(), [&](const auto& facet) {
                           // Check if the projection of the segment onto
                           // the facet intersects the facet
                           float t = -(facet.normal * start + facet.offset) /
                                     (facet.normal * (end - start));
                           pcl::PointXYZ intersection_point =
                               start + t * (end - start);

                           return facet.normal * intersection_point +
                                      facet.offset <=
                                  0;
                       }) >= 2;
        }
    }

    bool is_valid_movement(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& current_point, const pcl::PointXYZ& dest_point,
        const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, float scale_factor,
        const std::vector<std::unique_ptr<geos::geom::Geometry>>& polygons) {
        float jump_distance = scale_factor;
        float radius = 0.1;

        // return std::all_of(convexhulls.begin(), convexhulls.end(),
        //                    [&](const auto& convexhull) {
        //                        return !check_convexhull_intersection(
        //                            current_point, dest_point,
        //                            convexhull);
        //                    });

        return std::all_of(polygons.begin(), polygons.end(),
                           [&](const auto& polygon) {
                               return !check_polygon_intersection(
                                   current_point, dest_point, polygon);
                           });
    }

    std::tuple<pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ, float>
    get_plane_from_3_points(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2,
                            const pcl::PointXYZ& p3) {
        pcl::PointXYZ span_v1 = p3 - p1;
        auto span_v1_gs = span_v1 / sqrt(span_v1 * span_v1);

        pcl::PointXYZ span_v2 = p2 - p1;
        auto span_v2_gs = span_v2 - (span_v2 * span_v1_gs) * span_v1_gs;
        span_v2_gs = span_v2_gs / sqrt(span_v2_gs * span_v2_gs);

        // span_v2_gs = span_v2_gs / sqrt(span_v2_gs * span_v2_gs);

        auto cp = cross_product(span_v1_gs, span_v2_gs);
        // auto cp = cross_product(span_v1, span_v2);
        auto d = cp * p3;

        return {cp, span_v1_gs, span_v2_gs, d};
        // return {cp, span_v1, span_v2, d};
    }

    std::vector<pcl::PointIndices> get_clusters(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
        cv::Mat points(cloud->size(), 1, CV_32FC3);
        cv::Mat labels;
        std::vector<cv::Point3f> centers;

        for (int i = 0; i < cloud->size(); ++i) {
            const auto& p = (*cloud)[i];
            points.at<cv::Point3f>(i) = cv::Point3f(p.x, p.y, p.z);
        }

        double compactness =
            cv::kmeans(points, 100, labels,
                       cv::TermCriteria(cv::TermCriteria::EPS, 10, 0.2), 10,
                       cv::KMEANS_PP_CENTERS, centers);

        std::vector<pcl::PointIndices> cluster_indices(centers.size());

        for (int i = 0; i < cloud->size(); ++i) {
            cluster_indices[labels.at<int>(i)].indices.push_back(i);
        }

        cluster_indices.erase(
            std::remove_if(cluster_indices.begin(), cluster_indices.end(),
                           [](const auto& indices) {
                               return indices.indices.size() < 40;
                           }),
            cluster_indices.end());

        auto el =
            std::min_element(cluster_indices.begin(), cluster_indices.end(),
                             [](const auto& a, const auto& b) {
                                 return a.indices.size() < b.indices.size();
                             });

        std::cout << el->indices.size() << std::endl;

        std::cout << "Found " << cluster_indices.size() << " clusters"
                  << std::endl;

        int j = 0;
        for (const auto& cluster : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
                new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : cluster.indices) {
                cloud_cluster->push_back((*cloud)[idx]);
            }  //*
            cloud_cluster->is_dense = true;

            pcl::io::savePCDFileASCII("test_pcd" + std::to_string(j++) + ".pcd",
                                      *cloud_cluster);
        }
        return cluster_indices;
    }

    std::tuple<Eigen::MatrixXf, float, float> get_distances(
        const std::vector<pcl::PointXYZ>& v1,
        const std::vector<pcl::PointXYZ>& v2) {
        float min_val = std::numeric_limits<float>::max();
        float sum_non_zero = 0;
        float count_non_zero = 0;

        Eigen::MatrixXf dist_mat(v1.size(), v2.size());

        for (int i = 0; i < v1.size(); ++i) {
            for (int j = 0; j < v2.size(); ++j) {
                const auto diff = v2[j] - v1[i];
                const float dist = sqrt(diff * diff);
                if (dist != 0) {
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
                                                std::size_t amount) {
        std::vector<std::size_t> indices;
        indices.reserve(amount);

        std::random_device rd;
        std::mt19937 gen(rd());

        std::uniform_int_distribution<std::size_t> dis(0, vec_length);

        std::generate_n(std::back_inserter(indices), amount,
                        [&] { return dis(gen); });

        return indices;
    }

    void sort_matrix_columns(Eigen::MatrixXf& mat) {
        for (Eigen::Index col = 0; col < mat.cols(); ++col) {
            std::vector<float> col_vec(mat.rows());

            for (Eigen::Index row = 0; row < mat.rows(); ++row) {
                col_vec[row] = mat(row, col);
            }

            std::sort(col_vec.begin(), col_vec.end());

            for (Eigen::Index row = 0; row < mat.rows(); ++row) {
                mat(row, col) = col_vec[row];
            }
        }
    }

    std::vector<float> sum_rows(const Eigen::VectorXf& mat) {
        std::vector<float> sum_vec(mat.cols());

        for (Eigen::Index i = 0; i < mat.cols(); ++i) {
            sum_vec[i] = mat.col(i).sum();
        }

        return sum_vec;
    }

    pcl::PointXYZ robust_median(const std::vector<pcl::PointXYZ>& points,
                                int k) {
        auto random_indices = get_random_indices(
            points.size(), static_cast<std::size_t>(3 * sqrt(points.size())));
        std::sort(random_indices.begin(), random_indices.end());
        random_indices.erase(
            std::unique(random_indices.begin(), random_indices.end()),
            random_indices.end());
        std::vector<pcl::PointXYZ> subset_points(random_indices.size());

        std::transform(random_indices.begin(), random_indices.end(),
                       subset_points.begin(),
                       [&](const auto& index) { return points[index]; });

        auto distances =
            std::get<0>(get_distances(subset_points, subset_points));

        sort_matrix_columns(distances);

        const auto dists_block = distances.block(
            0, 0,
            static_cast<int>(std::round(15 / (16 * k))) * random_indices.size(),
            distances.cols());

        std::vector<float> sums(dists_block.cols());

        for (Eigen::Index col = 0; col < dists_block.cols(); ++col) {
            sums[col] = dists_block.col(col).sum();
        }

        return points[random_indices[std::distance(
            sums.begin(), std::min_element(sums.begin(), sums.end()))]];
    }

    void delaunay_greedy_based_picking(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
        auto centers = recursive_robust_median_clustering(cloud, 20);
    }

    std::vector<pcl::PointXYZ> recursive_robust_median_clustering(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int k) {
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

        while (Q.size() > 10 * static_cast<int>(std::log(points.size()))) {
            Q_temp.setLinSpaced(static_cast<Eigen::Index>(Q.size()) - 1, 0,
                                Q.size() - 1);
            for (std::size_t i = 0;
                 i < static_cast<std::size_t>(std::log(Q.size())); ++i) {
                auto points_subset =
                    get_indices(points, get_indices(Q, Q_temp));
                auto center = robust_median(points_subset, k);
                std::vector<float> dist_from_center(points_subset.size());

                std::transform(points_subset.begin(), points_subset.end(),
                               dist_from_center.begin(), [&](const auto& p) {
                                   const auto new_p = p - center;
                                   return sqrt(new_p * new_p);
                               });

                auto idxs = argsort(dist_from_center);
                idxs = std::vector(
                    idxs.begin(),
                    idxs.begin() +
                        static_cast<std::size_t>(dist_from_center.size() / 2));

                Q_temp = get_indices(Q_temp, idxs);
            }

            auto points_for_mean = get_indices(points, get_indices(Q, Q_temp));
            centers.push_back(
                std::accumulate(
                    points_for_mean.begin(), points_for_mean.end(),
                    pcl::PointXYZ{0, 0, 0},
                    [](const pcl::PointXYZ& sum, const pcl::PointXYZ& p) {
                        return sum + p;
                    }) /
                static_cast<float>(points_for_mean.size()));
            Q = get_all_indices_except(Q, Q_temp);
        }

        auto points_for_final_mean = get_indices(points, Q);
        centers.push_back(
            std::accumulate(points_for_final_mean.begin(),
                            points_for_final_mean.end(), pcl::PointXYZ{0, 0, 0},
                            [](const pcl::PointXYZ& sum,
                               const pcl::PointXYZ& p) { return sum + p; }) /
            static_cast<float>(points_for_final_mean.size()));

        return centers;
    }

    Eigen::VectorXf gradient(const Eigen::VectorXf& v) {
        if (v.size() <= 1) return v;

        Eigen::VectorXf res(v.size());
        for (int j = 0; j < v.size(); j++) {
            int j_left = j - 1;
            int j_right = j + 1;
            if (j_left < 0) {
                j_left = 0;  // use your own boundary handler
                j_right = 1;
            }
            if (j_right >= v.size()) {
                j_right = v.size() - 1;
                j_left = j_right - 1;
            }
            // gradient value at position j
            auto dist_grad = (v(j_right) - v(j_left)) / 2.0;
            res(j) = dist_grad;
        }
        return res;
    }

    pcl::PointXYZ get_random_point_on_plane(
        const pcl::PointXYZ& start, const pcl::PointXYZ& point_of_interest,
        const pcl::PointXYZ& u1, const pcl::PointXYZ& u2,
        const pcl::PointXYZ& cp) {
        auto start_proj = u1 * (u1 * start) + u2 * (u2 * start);
        auto end_proj =
            u1 * (u1 * point_of_interest) + u2 * (u2 * point_of_interest);

        auto normal_to_plane =
            cross_product(cp / sqrt(cp * cp), start_proj - end_proj);

        auto diff = end_proj - start_proj;

        std::random_device rd;
        std::mt19937 gen(rd());

        std::uniform_real_distribution<float> dis_1(0, 2 * sqrt((diff * diff)));
        std::uniform_real_distribution<float> dis_2(0, sqrt((diff * diff)));

        return dis_1(gen) * (end_proj - start_proj) +
               dis_2(gen) * normal_to_plane;
    }

    pcl::PointXYZ get_random_point_on_plane(const pcl::PointXYZ& span_v1,
                                            const pcl::PointXYZ& span_v2) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-30, 30);

        return dis(gen) * span_v1 + dis(gen) * span_v2;
    }

    pcl::PointXYZ get_random_point_on_plane(const pcl::PointXYZ& cp, float d) {
        std::random_device rd;
        std::mt19937 gen(rd());

        std::uniform_real_distribution<float> dis(-100, 100);
        auto x = dis(gen);
        auto y = dis(gen);
        return {x, y, (-d - cp.x * x - cp.y * y) / (cp.z == 0 ? 0.001f : cp.z)};
    }

    pcl::PointXYZ cross_product(const pcl::PointXYZ& v_a,
                                const pcl::PointXYZ& v_b) {
        return {v_a.y * v_b.z - v_a.z * v_b.y, -(v_a.x * v_b.z - v_a.z * v_b.x),
                v_a.x * v_b.y - v_a.y * v_b.x};
    }

    std::vector<pcl::PointXYZ> load_path_from_file(
        const std::filesystem::path& location_file_path) {
        std::vector<pcl::PointXYZ> path_to_unknown;

        std::ifstream fin(location_file_path);

        std::vector<float> values;
        if (fin.good()) {
            float value = 0;
            while (fin >> value) {
                values.push_back(value);
                if (values.size() == 3) {
                    path_to_unknown.emplace_back(values[0], values[1],
                                                 values[2]);
                    values.clear();
                }
            }
        }

        return path_to_unknown;
    }

    void save_points_to_file(const std::vector<pcl::PointXYZ>& points,
                             const std::filesystem::path& location_file_path) {
        std::ofstream file_of_path(location_file_path);

        for (const auto& point : points) {
            file_of_path << point.x << " " << point.y << " " << point.z
                         << std::endl;
        }

        file_of_path.close();
    }

    void save_points_to_file(
        const std::vector<Eigen::Matrix<double, 3, 1>>& points,
        const std::filesystem::path& location_file_path) {
        std::ofstream file_of_path(location_file_path);

        for (const auto& point : points) {
            file_of_path << point.x() << " " << point.y() << " " << point.z()
                         << std::endl;
        }

        file_of_path.close();
    }

    void save_points_to_file(const std::vector<cv::Point3f>& points,
                             const std::filesystem::path& location_file_path) {
        std::ofstream file_of_path(location_file_path);

        for (const auto& point : points) {
            file_of_path << point.x << " " << point.y << " " << point.z
                         << std::endl;
        }

        file_of_path.close();
    }

}  // namespace Auxilary
