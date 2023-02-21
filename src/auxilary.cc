#include "auxilary.hpp"

#include <libqhull_r/libqhull_r.h>
#include <libqhullcpp/PointCoordinates.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullPoint.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/RboxPoints.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ml/kmeans.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <random>
#include <string>
#include <unordered_map>
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

    // custom specialization of std::hash can be injected in namespace std

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

    std::vector<ConvexHullEquations> get_convexhulls(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const std::vector<pcl::PointIndices>& cluster_indices) {
        std::vector<ConvexHullEquations> convexhulls;

        for (const auto& i : cluster_indices) {
            std::vector<double> cluster_points;
            orgQhull::PointCoordinates coords(3, "");
            std::for_each(i.indices.begin(), i.indices.end(),
                          [&](const auto& p_index) {
                              const auto& p = (*cloud)[p_index];
                              cluster_points.push_back(p.x);
                              cluster_points.push_back(p.y);
                              cluster_points.push_back(p.z);
                          });

            coords.append(cluster_points);

            orgQhull::Qhull qhull;

            qhull.runQhull(coords.comment().c_str(), coords.dimension(),
                           coords.count(), coords.coordinates(), "");

            ConvexHullEquations current_convexhull;
            std::for_each(
                qhull.facetList().begin(), qhull.facetList().end(),
                [&](const auto& f) {
                    if (f.isGood()) {
                        const double* normal_ptr = f.getFacetT()->normal;
                        current_convexhull.facets_planes.push_back(
                            {pcl::PointXYZ(static_cast<float>(normal_ptr[0]),
                                           static_cast<float>(normal_ptr[1]),
                                           static_cast<float>(normal_ptr[2])),
                             f.getFacetT()->offset});
                    }
                });

            convexhulls.push_back(current_convexhull);
        }

        return convexhulls;
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
        return std::all_of(
            convexhull.facets_planes.begin(), convexhull.facets_planes.end(),
            [&](const auto& facet) {
                float start_dot_product = facet.normal * start;
                float end_dot_product = facet.normal * end;
                if ((start_dot_product >= 0 && end_dot_product >= 0) ||
                    (start_dot_product <= 0 && end_dot_product <= 0)) {
                    return false;
                }

                // Check if the projection of the segment onto the facet normal
                // intersects the facet
                float t = -(facet.normal * start + facet.offset) /
                          (facet.normal * (end - start));
                pcl::PointXYZ intersection_point = start + t * (end - start);

                return facet.normal * intersection_point + facet.offset <= 0;
            });
    }

    bool is_valid_movement(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const pcl::PointXYZ& current_point, const pcl::PointXYZ& dest_point,
        const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, float scale_factor,
        const std::vector<ConvexHullEquations>& convexhulls) {
        float jump_distance = scale_factor;
        float radius = 0.1;

        return std::all_of(convexhulls.begin(), convexhulls.end(),
                           [&](const auto& convexhull) {
                               return !check_convexhull_intersection(
                                   current_point, dest_point, convexhull);
                           });

        std::vector<pcl::PointXYZ> v_points_on_line =
            get_points_on_line(current_point, dest_point, jump_distance);

        return std::all_of(v_points_on_line.begin(), v_points_on_line.end(),
                           [&](const auto& p) {
                               // return radius_search(cloud, p, radius, kdtree)
                               // == 0;
                               return std::all_of(
                                   convexhulls.begin(), convexhulls.end(),
                                   [&](const auto& convexhull) {
                                       return !check_convexhull_intersection(
                                           p, convexhull);
                                   });
                           });
    }

    std::pair<pcl::PointXYZ, float> get_plane_from_3_points(
        const pcl::PointXYZ& p1, const pcl::PointXYZ& p2,
        const pcl::PointXYZ& p3) {
        pcl::PointXYZ span_v1 = p3 - p1;
        pcl::PointXYZ span_v2 = p2 - p1;

        auto cp = cross_product(span_v1, span_v2);
        auto d = cp * p3;

        return {cp, d};
    }

    std::vector<pcl::PointIndices> get_clusters(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree) {
        pcl::io::savePCDFileASCII("clean_cloud.pcd", *cloud);

        cv::Mat points(cloud->size(), 1, CV_32FC3);
        cv::Mat labels;
        std::vector<cv::Point3f> centers;

        for (int i = 0; i < cloud->size(); ++i) {
            const auto& p = (*cloud)[i];
            points.at<cv::Point3f>(i) = cv::Point3f(p.x, p.y, p.z);
        }

        double compactness = cv::kmeans(
            points, 10, labels,
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                             10, 1.0),
            3, cv::KMEANS_PP_CENTERS, centers);

        std::vector<pcl::PointIndices> cluster_indices(centers.size());

        for (int i = 0; i < cloud->size(); ++i) {
            cluster_indices[labels.at<int>(i)].indices.push_back(i);
        }

        cluster_indices.erase(
            std::remove_if(
                cluster_indices.begin(), cluster_indices.end(),
                [](const auto& indices) { return indices.indices.size() < 6; }),
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

    pcl::PointXYZ get_random_point_on_plane(const pcl::PointXYZ& cp, float d) {
        std::random_device rd;
        std::mt19937 gen(rd());

        std::uniform_real_distribution<float> dis(-30, 30);
        auto x = dis(gen);
        auto y = dis(gen);
        return {x, y, (d - cp.x * x - cp.y * y) / (cp.z == 0 ? 0.001f : cp.z)};
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
