#include <geos/triangulate/tri/Tri.h>
#include <geos/triangulate/tri/TriList.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <algorithm>
#include <cstddef>
#include <future>

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
#include <geos/triangulate/polygon/ConstrainedDelaunayTriangulator.h>
#include <opencv2/core/hal/interface.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

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

using namespace Auxilary;
using namespace std::chrono_literals;
void visualizer_cloud_and_path(

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float ScaleFactor,
    std::vector<std::vector<pcl::PointXYZ>> paths

) {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

    auto cluster_indices = get_clusters(cloud);
    auto convexhulls = get_convexhulls(cloud, cluster_indices);

    std::vector<geos::triangulate::tri::TriList<geos::triangulate::tri::Tri>>
        tri_vec;

    std::transform(
        convexhulls.begin(), convexhulls.end(), std::back_inserter(tri_vec),
        [](const auto& convexhull) {
            const auto* poly =
                dynamic_cast<geos::geom::Polygon*>(convexhull.get());
            geos::triangulate::tri::TriList<geos::triangulate::tri::Tri>
                tri_list;

            geos::triangulate::polygon::ConstrainedDelaunayTriangulator::
                triangulatePolygon(poly, tri_list);

            return tri_list;
        });

    // std::vector<pcl::Polygon> polygons;

    for (int i = 0; i < convexhulls.size(); ++i) {
        const auto& p = convexhulls[i];
        // const auto coords_size = p.size() * 3;
        const auto coords_size = p->getCoordinates()->getSize();
        const auto coords = p->getCoordinates();

        pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

        polygon_cloud->resize(coords_size);

        // pcl::Vertices convexhull_indices;
        // convexhull_indices.vertices.resize(p->getCoordinates()->size());
        // std::iota(convexhull_indices.vertices.begin(),
        //           convexhull_indices.vertices.end(), 0);

        for (int j = 0; j < p->getCoordinates()->size(); ++j) {
            const auto& coord = coords->getAt(j);

            polygon_cloud->points[j].x = coord.x;
            polygon_cloud->points[j].y = coord.y;
            polygon_cloud->points[j].z = coord.z;
        }

        // pcl::PCLPointCloud2::Ptr polygon_cloud2(new pcl::PCLPointCloud2);
        // p_meshes.emplace_back();
        // pcl::toPCLPointCloud2(*polygon_cloud, *polygon_cloud2);

        // p_meshes[i].cloud = *polygon_cloud2;
        // p_meshes[i].polygons.push_back(convexhull_indices);
        // p_meshes[i].header = pcl::PCLHeader();

        // std::cout << p_meshes[i] << std::endl;

        // viewer->addPolygonMesh(p_meshes[i], "mesh" + std::to_string(i));

        // viewer->addPolygon<pcl::PointXYZ>(polygon_cloud, 0.1, 0.2, 0.9,
        //                                   std::to_string(i), 0);
    }

    int index = 0;
    for (std::size_t i = 0; i < paths.size(); ++i) {
        for (std::size_t j = 0; j < paths[i].size(); ++j) {
            std::stringstream ss;
            ss << "PointNavigatePath" << i << j;
            viewer->addSphere(paths[i][j], 0.08, 0.1, 0.2, 0.9, ss.str());
            if (j > 0) {
                viewer->addLine(
                    paths[i][j - 1], paths[i][j], 0.1, 0.2, 0.9,
                    "arrow" + std::to_string(i) + std::to_string(j));
            }
            // index++;
        }
        // if (!paths[i].empty()) {
        //     viewer->addSphere(paths[i].front(), 0.05, 0.9, 0.2, 0.2,
        //                       "Starting Point" + std::to_string(i));
        // }

        int data = 0;
    }
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

int main(int argc, char** argv) {  // TODO : Redo the point Enter !!!
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

    // The resulting cloud_out contains all points of cloud_in that have an
    // average distance to their 8 nearest neighbors that is below the computed
    // threshold Using a standard deviation multiplier of 1.0 and assuming the
    // average distances are normally distributed there is a 84.1% chance that a
    // point will be an inlier
    // indices_rem = sorfilter.getRemovedIndices();

    const float scale_factor = std::stof(argv[4]);

    // Explorer explorer(cloud);
    // explorer.set_plane_of_flight((*plane)[0], (*plane)[1], (*plane)[2]);
    // visualizer_cloud_and_path(
    //     cloud, scale_factor,
    //     std::vector<pcl::PointXYZ>{(*plane)[0], (*plane)[1], (*plane)[2]});

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_point(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud);
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter(
    //     true);  // Initializing with true will allow us to extract the
    //             // removed
    // // indices
    // sorfilter.setInputCloud(cloud);
    // sorfilter.setMeanK(8);
    // sorfilter.setStddevMulThresh(1.0);
    // sorfilter.filter(*cloud_out);

    // cloud = cloud_out;
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *plane);
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[3], *start_point);
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[5], *target_point);
    std::shared_ptr<pcl::PointXYZ> p(new pcl::PointXYZ((*target_point)[0]));
    Explorer explorer(cloud);
    explorer.set_plane_of_flight((*plane)[0], (*plane)[1], (*plane)[2]);
    for (int i = 0; i < 30; ++i) {
        auto path =
            explorer.get_points_to_unknown((*start_point)[0], 0.001, nullptr);
        auto graph = explorer.get_last_graph();
        std::cout << "GSIZE " << graph.size() << std::endl;
        visualizer_cloud_and_path(cloud, scale_factor, {path});
        visualizer_cloud_and_path(cloud, scale_factor, {graph});

        visualizer_cloud_and_path(cloud, scale_factor, explorer.best_paths);
    }

    // visualizer_cloud_and_path(cloud, scale_factor, path);
    // visualizer_cloud_and_path(
    //     cloud, scale_factor,
    //     explorer.get_points_to_unknown((*start_point)[0], scale_factor, p));
    // visualizer_cloud_and_path(cloud, scale_factor, path);
    // visualizer_cloud_and_path(
    //     cloud, scale_factor,
    //     explorer.get_points_to_unknown((*start_point)[0], scale_factor));
    // visualizer_cloud_and_path(cloud, 0.1, explorer.get_last_graph());

    return 0;
}
