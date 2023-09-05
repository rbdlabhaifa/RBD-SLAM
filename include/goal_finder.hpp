#ifndef GOAL_FINDER_H_
#define GOAL_FINDER_H_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace goal_finder
{

class DataReader
{
  public:
    std::vector<std::vector<double>> read_file(const std::string &file_path);

  private:
    std::string get_file_extension(const std::string &file_path);
    std::vector<std::vector<double>> read_xyz(const std::string &file_path);
    std::vector<std::vector<double>> read_csv(const std::string &file_path);
};

class DataProcessor
{
  public:
    std::vector<std::vector<double>>
    calculate_zscores(const std::vector<std::vector<double>> &data_points);
    std::vector<std::vector<double>>
    clean_data(const std::vector<std::vector<double>> &data_points,
               double zscore_threshold);

  private:
    std::vector<bool>
    find_outliers(const std::vector<std::vector<double>> &zscores,
                  double threshold);
};

class DataAnalyzer
{
  public:
    std::vector<double>
    find_middle_point(const std::vector<std::vector<double>> &data_points);
    std::vector<double>
    calculate_variances(const std::vector<std::vector<double>> &data_points);
    std::vector<size_t>
    select_best_dimensions(const std::vector<std::vector<double>> &data_points);
    std::vector<std::vector<double>>
    project_to_dimensions(const std::vector<std::vector<double>> &data_points,
                          const std::vector<size_t> &dimensions);

    /* std::vector<double> calculate_average_distances(const
    std::vector<std::vector<double>>& data_points, const std::vector<double>&
    drone_pos) { size_t num_angles = 360; std::vector<double>
    avg_distances(num_angles, std::numeric_limits<double>::quiet_NaN());
        std::vector<int> count_datapoints_per_angle(num_angles, 0);

        for (size_t i = 0; i < num_angles; ++i) {
            double angle_min = i * M_PI / 180.0;
            double angle_max = (i + 1) * M_PI / 180.0;
            std::vector<size_t> indices;

            for (size_t j = 0; j < data_points.size(); ++j) {
                double angle = std::atan2(data_points[j][1] - drone_pos[1],
    data_points[j][0] - drone_pos[0]); angle = std::fmod(angle + 2 * M_PI, 2 *
    M_PI);

                if (angle >= angle_min && angle < angle_max) {
                    indices.push_back(j);
                }
            }

            size_t count = indices.size();
            count_datapoints_per_angle[i] = count;

            if (count > 1) {
                double sum_distances = 0.0;
                for (size_t index : indices) {
                    sum_distances += std::sqrt(std::pow(data_points[index][0] -
    drone_pos[0], 2) + std::pow(data_points[index][1] - drone_pos[1], 2));
                }
                avg_distances[i] = sum_distances / count;
            }
        }

        return avg_distances;
    } */

    std::vector<size_t>
    count_nan_dist_neighbors(const std::vector<size_t> &nan_indices,
                             const std::vector<double> &avg_distances,
                             size_t buffer_size);

    double calculate_angle(const std::vector<double> &point1,
                           const std::vector<double> &point2);

    std::vector<double> calculate_average_distances(
        const std::vector<std::vector<double>> &data_points,
        const std::vector<double> &drone_pos);

    std::pair<std::vector<double>, std::vector<double>>
    find_exit(const std::vector<std::vector<double>> &datapoints,
              const std::vector<double> &drone_pos,
              const std::vector<double> &avg_distances);
};

std::vector<double> Find_Goal(std::vector<std::vector<double>> map_points);
}; // namespace goal_finder

#endif /* end of NAVIGATOR_H_ */