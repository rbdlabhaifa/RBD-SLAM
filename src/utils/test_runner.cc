#include <eigen3/Eigen/Dense>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "goal_finder.hpp"
#include "pcl_operations.hpp"

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

int main(int argc, char **argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr start_point(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *plane) ==
        -1) //* load the file
    {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[3], *start_point) ==
        -1) //* load the file
    {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    goal_finder::DataReader map_reader;
    std::vector<std::vector<double>> vec_map = map_reader.read_file(argv[1]);

    pcl::PointXYZ start_p = (*start_point)[0];
    Eigen::Vector3d start_pos{start_p.x, start_p.y, start_p.z};

    Eigen::Vector3d exit = goal_finder::Find_Goal(
        vec_map, start_pos, (*plane)[0], (*plane)[1], (*plane)[2], 1.5);

    std::cout << "EXIT: " << exit << std::endl;

    std::vector<pcl::PointXYZ> save;
    pcl::PointXYZ s_p{
        static_cast<float>(exit.x()),
        static_cast<float>(exit.y()),
        static_cast<float>(exit.z()),
    };
    save.push_back(s_p);

    save_points_to_file(save, "../exit.xyz");

    return 0;
}
