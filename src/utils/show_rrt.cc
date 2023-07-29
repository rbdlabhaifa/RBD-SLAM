#include <geos/triangulate/polygon/ConstrainedDelaunayTriangulator.h>
#include <geos/triangulate/tri/Tri.h>
#include <geos/triangulate/tri/TriList.h>
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
#include "path_builder.hpp"

using namespace Auxilary;
using namespace std::chrono_literals;
void visualizer_cloud_and_path(

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float ScaleFactor,
    std::vector<std::vector<pcl::PointXYZ>> paths

) {
	std::cout<< " in visualizer" << std:: endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    
    	std::cout<< " segfault 1" << std:: endl;

    int index = 0;
    for (std::size_t i = 0; i < paths.size(); ++i) {
        for (std::size_t j = 0; j < paths[i].size(); ++j) {
            std::stringstream ss;
            ss << "PointNavigatePath" << i << j;
            viewer->addSphere(paths[i][j], ScaleFactor, 0.1, 0.2, 0.9,
                              ss.str());
            if (j > 0) {
                viewer->addLine(
                    paths[i][j - 1], paths[i][j], 0.1, 0.2, 0.9,
                    "arrow" + std::to_string(i) + std::to_string(j));
            }
            // index++;
        }
        
        std::cout<< " segfault 2" << std:: endl;

    }
    while (!viewer->wasStopped()) {
        viewer->spin();
        std::this_thread::sleep_for(5ms);
    }
    std::cout<< " segfault 3" << std:: endl;
}

void viewRRTFunction(pcl::visualization::PCLVisualizer::Ptr viewer) {
	std::cout<< " in view rrt funvtion" << std:: endl;
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}
pcl::visualization::PCLVisualizer::Ptr shapesVis(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    std::cout<< " in shapesvis" << std:: endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    return (viewer);
}

int main(int argc, char** argv) {  // TODO : Redo the point Enter !!!
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr start_point(
        new pcl::PointCloud<pcl::PointXYZ>);

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

    const float scale_factor = std::stof(argv[4]);

    Explorer explorer(cloud);
    explorer.set_plane_of_flight((*plane)[0], (*plane)[1], (*plane)[2]);
    for (int i = 0; i < 30; ++i) {
        auto path = explorer.get_points_to_unknown((*start_point)[0]);
        auto graph = explorer.get_last_graph();
        std::cout << "GSIZE " << graph.size() << std::endl;
        visualizer_cloud_and_path(cloud, scale_factor, {path});
        visualizer_cloud_and_path(cloud, scale_factor, {graph});

        visualizer_cloud_and_path(cloud, scale_factor, explorer.best_paths);
    }

    return 0;
}
