#include "auxilary.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <vector>

using namespace Auxilary;

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

    pcl::PointXYZ operator-(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
        return pcl::PointXYZ(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
    }

    pcl::PointXYZ operator/(const pcl::PointXYZ& p, float d) {
        if (d == 0) return p;

        return pcl::PointXYZ(p.x / d, p.y / d, p.z / d);
    }

    float norm(const pcl::PointXYZ& p) {
        return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    float operator*(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
        return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
    }

    pcl::PointXYZ operator*(const pcl::PointXYZ& p, float a) {
        return pcl::PointXYZ(p.x * a, p.y * a, p.z * a);
    }
    pcl::PointXYZ operator*(float a, const pcl::PointXYZ& p) {
        return pcl::PointXYZ(p.x * a, p.y * a, p.z * a);
    }

    pcl::PointXYZ operator+(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
        return pcl::PointXYZ(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
    }

    bool operator==(const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
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
            points_on_line.push_back(pcl::PointXYZ(start + hat_p * i));
        }

        return points_on_line;
    }

    bool is_valid_movement(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                           const pcl::PointXYZ& current_point,
                           const pcl::PointXYZ& dest_point,
                           const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                           float scale_factor) {
        float jump_distance = 0.1 * scale_factor;
        float radius = 0.25 * scale_factor;

        std::vector<pcl::PointXYZ> v_points_on_line =
            get_points_on_line(current_point, dest_point, jump_distance);

        return std::all_of(
            v_points_on_line.begin(), v_points_on_line.end(), [&](auto p) {
                return radius_search(cloud, p, radius, kdtree) <= 5;
            });
    }

    pcl::PointXYZ get_random_point_on_plane_def_by_3_points(
        const pcl::PointXYZ& p1, const pcl::PointXYZ& p2,
        const pcl::PointXYZ& p3) {
        pcl::PointXYZ span_v1 = p3 - p1;
        pcl::PointXYZ span_v2 = p3 - p2;

        std::random_device rd;
        std::mt19937 gen(rd());

        std::uniform_real_distribution<> dis(-30, 30);

        return pcl::PointXYZ(dis(gen) * span_v1 + dis(gen) * span_v2);
    }

    pcl::PointXYZ cross_product(const pcl::PointXYZ& v_a,
                                const pcl::PointXYZ& v_b) {
        return pcl::PointXYZ(v_a.y * v_b.z - v_a.z * v_b.y,
                             -(v_a.x * v_b.z - v_a.z * v_b.x),
                             v_a.x * v_b.y - v_a.y * v_b.x);
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

    void save_path_to_file(const std::vector<pcl::PointXYZ>& path,
                           const std::filesystem::path& location_file_path) {
        std::ofstream file_of_path(location_file_path);

        for (auto point : path)
            file_of_path << point.x << " " << point.y << " " << point.z
                         << std::endl;

        file_of_path.close();
    }

    void save_points_to_file(
        const std::vector<Eigen::Matrix<double, 3, 1>>& points,
        const std::filesystem::path& location_file_path) {
        std::ofstream file_of_path(location_file_path);

        for (auto point : points)
            file_of_path << point.x() << " " << point.y() << " " << point.z()
                         << std::endl;

        file_of_path.close();
    }

    void save_points_to_file(const std::vector<cv::Point3f>& points,
                             const std::filesystem::path& location_file_path) {
        std::ofstream file_of_path(location_file_path);

        for (auto point : points)
            file_of_path << point.x << " " << point.y << " " << point.z
                         << std::endl;

        file_of_path.close();
    }

}  // namespace Auxilary
