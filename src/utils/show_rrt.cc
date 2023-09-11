#include <geos/triangulate/polygon/ConstrainedDelaunayTriangulator.h>
#include <geos/triangulate/tri/Tri.h>
#include <geos/triangulate/tri/TriList.h>
#include <nlohmann/json.hpp>
#include <opencv2/core/hal/interface.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>
#include <cstddef>
#include <future>
#include <iterator>
#include <memory>
#include <numeric>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "auxilary.hpp"
#include "explorer.hpp"
#include "goal_finder.hpp"
#include "path_builder.hpp"

using namespace Auxilary;
using namespace std::chrono_literals;
void visualizer_cloud_and_path(

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float ScaleFactor,
    std::vector<std::vector<pcl::PointXYZ>> paths

)
{
    std::cout << " in visualizer" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

    std::cout << " segfault 1" << std::endl;

    int index = 0;
    for (std::size_t i = 0; i < paths.size(); ++i)
    {
        for (std::size_t j = 0; j < paths[i].size(); ++j)
        {
            std::stringstream ss;
            ss << "PointNavigatePath" << i << j;
            viewer->addSphere(paths[i][j], ScaleFactor, 0.1, 0.2, 0.9,
                              ss.str());
            if (j > 0)
            {
                viewer->addLine(paths[i][j - 1], paths[i][j], 0.1, 0.2, 0.9,
                                "arrow" + std::to_string(i) +
                                    std::to_string(j));
            }
            // index++;
        }

        std::cout << " segfault 2" << std::endl;
    }
    while (!viewer->wasStopped())
    {
        viewer->spin();
        std::this_thread::sleep_for(5ms);
    }
    std::cout << " segfault 3" << std::endl;
}

void viewRRTFunction(pcl::visualization::PCLVisualizer::Ptr viewer)
{
    std::cout << " in view rrt funvtion" << std::endl;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}
pcl::visualization::PCLVisualizer::Ptr
shapesVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    std::cout << " in shapesvis" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    return (viewer);
}

int main(int argc, char **argv)
{ // TODO : Redo the point Enter !!!
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr start_point(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) ==
        -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *plane) ==
        -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[3], *start_point) ==
        -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return -1;
    }

    const float scale_factor = std::stof(argv[4]);

    std::ifstream programData("/home/ido/rbd/rbd-slam/RBD-SLAM/config.json");
    nlohmann::json data;
    programData >> data;
    programData.close();

    // Main config:
    std::string vocabulary_path = data["Vocabulary_Path"];
    std::string calibration_path = data["calibration_path"];
    std::string map_path = data["map_path"];
    std::string data_save_dir = data["data_save_dir"];

    bool fake_drone = data["fake_drone"];
    bool use_webcam = data["use_webcam"];
    bool offline_mode = data["offline_mode"];

    // smi-magic numbers
    // goal_finder
    float find_exit_dist_scalar = data["find_exit_dist_scalar"];
    // rrt - path builder
    int rrt_path_size = data["rrt_path_size"];
    int rrt_ring_point_amount = data["rrt_ring_point_amount"];
    float rrt_jump_size = data["rrt_jump_size"];
    float rrt_ring_size_scalar = data["rrt_ring_size_scalar"];
    float rrt_goal_threshold = data["rrt_goal_threshold"];

    Explorer explorer(cloud);
    explorer.set_plane_of_flight((*plane)[0], (*plane)[1], (*plane)[2]);

    auto [k_p1, k_p2, k_p3] = explorer.get_plane_of_flight();
    Eigen::Vector3d vec_last_loc{(*start_point)[0].x, (*start_point)[0].y,
                                 (*start_point)[0].z};

    std::vector<std::vector<double>> transformed_vec;

    for (pcl::PointXYZ point : *cloud)
    {
        transformed_vec.push_back(
            std::vector<double>{point.x, point.y, point.z});
    }

    auto goal_exit_point = goal_finder::Find_Goal(
        transformed_vec, vec_last_loc, k_p1, k_p2, k_p3, find_exit_dist_scalar);

    explorer.exit_point = pcl::PointXYZ(static_cast<float>(goal_exit_point[0]),
                                        static_cast<float>(goal_exit_point[1]),
                                        static_cast<float>(goal_exit_point[2]));

    auto path = explorer.get_points_to_exit(
        (*start_point)[0], rrt_goal_threshold, rrt_jump_size,
        rrt_ring_point_amount, rrt_ring_size_scalar);

    std::cout << std::endl;
    for (auto point : path)
        std::cout << point;
    std::cout << std::endl;

    std::cout << "\n\nstart:" << (*start_point)[0] << std::endl;
    std::cout << "exit: " << goal_exit_point.transpose() << "\n\n" << std::endl;

    auto graph = explorer.get_last_graph();
    std::cout << "GSIZE " << graph.size() << std::endl;
    visualizer_cloud_and_path(cloud, scale_factor, {path});
    // visualizer_cloud_and_path(cloud, scale_factor, {graph});

    // visualizer_cloud_and_path(cloud, scale_factor, explorer.best_paths);
    return 0;
}
