#ifndef GOAL_FINDER_H_
#define GOAL_FINDER_H_

#include "auxilary.hpp"
#include "eigen_operations.hpp"
#include "pcl_operations.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <pcl/PointIndices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>

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

Eigen::Vector3d Find_Goal(std::vector<std::vector<double>> map_points,
                          Eigen::Vector3d starting_pos,
                          pcl::PointXYZ &known_point1,
                          pcl::PointXYZ &known_point2,
                          pcl::PointXYZ &known_point3);
}; // namespace goal_finder

#endif /* end of NAVIGATOR_H_ */