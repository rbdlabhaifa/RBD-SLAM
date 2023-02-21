#include <pcl/filters/statistical_outlier_removal.h>

#include "auxilary.hpp"
#include "explorer.hpp"
#include "path_builder.hpp"
// #include "lemon/core.h"
// #include <chrono>
// #include <fstream>
// #include <iostream>
// #include <lemon/bfs.h>
// #include <lemon/list_graph.h>
// #include <lemon/maps.h>
// #include <list>
// #include <pcl/common/common_headers.h>
// #include <pcl/console/parse.h>
// #include <pcl/features/normal_3d.h>
#include <opencv2/core/hal/interface.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace Auxilary;
using namespace std::chrono_literals;
void visualizer_cloud_and_path(

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float ScaleFactor,
    std::vector<pcl::PointXYZ> path_to_unknown

) {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

    int index = 0;
    for (auto &point : path_to_unknown) {
        std::stringstream ss;
        ss << "PointNavigatePath" << index;
        viewer->addSphere(point, 0.05, 0.1, 0.2, 0.9, ss.str());
        index++;
    }
    if (!path_to_unknown.empty()) {
        viewer->addSphere(path_to_unknown.front(), 0.05, 0.9, 0.2, 0.2,
                          "Starting Point");
    }

    int data = 0;

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
        std::this_thread::sleep_for(5ms);
    }
}

void viewRRTFunction(pcl::visualization::PCLVisualizer::Ptr viewer) {
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}
pcl::visualization::PCLVisualizer::Ptr shapesVis(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ>
    // rgb(cloud); viewer->addPointCloud<pcl::PointXYZ>(cloud, rgb, "sample
    // cloud");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    return (viewer);
}

int main(int argc, char **argv) {  // TODO : Redo the point Enter !!!
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr start_point(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (argc < 5) {
        std::cout << "Please Enter pcd you want to analysis  e.g yam_data.pcd"
                  << std::endl;
        return -1;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) ==
        -1)  //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *plane) ==
        -1)  //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[3], *start_point) ==
        -1)  //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return -1;
    }

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter(
    //     true);  // Initializing with true will allow us to extract the
    //     removed
    //             // indices
    // sorfilter.setInputCloud(cloud);
    // sorfilter.setMeanK(std::stoi(argv[5]));
    // sorfilter.setStddevMulThresh(1.0);
    // sorfilter.filter(*cloud_out);
    // The resulting cloud_out contains all points of cloud_in that have an
    // average distance to their 8 nearest neighbors that is below the computed
    // threshold Using a standard deviation multiplier of 1.0 and assuming the
    // average distances are normally distributed there is a 84.1% chance that a
    // point will be an inlier
    // indices_rem = sorfilter.getRemovedIndices();

    const float scale_factor = std::stof(argv[4]);

    Explorer explorer(cloud);
    explorer.set_plane_of_flight((*plane)[0], (*plane)[1], (*plane)[2]);
    visualizer_cloud_and_path(
        cloud, scale_factor,
        std::vector<pcl::PointXYZ>{(*plane)[0], (*plane)[1], (*plane)[2]});

    std::shared_ptr<pcl::PointXYZ> p(
        new pcl::PointXYZ(0.575846, -1.2662781818181816, -0.08196430909090908));
    visualizer_cloud_and_path(cloud, scale_factor,
                              std::vector<pcl::PointXYZ>{*p});
    auto path =
        explorer.get_points_to_unknown((*start_point)[0], scale_factor, p);
    visualizer_cloud_and_path(cloud, scale_factor, path);
    visualizer_cloud_and_path(
        cloud, scale_factor,
        explorer.get_points_to_unknown((*start_point)[0], scale_factor));
    visualizer_cloud_and_path(cloud, scale_factor, path);
    visualizer_cloud_and_path(
        cloud, scale_factor,
        explorer.get_points_to_unknown((*start_point)[0], scale_factor));
    visualizer_cloud_and_path(cloud, 0.1, explorer.get_last_graph());

    return 0;
}
