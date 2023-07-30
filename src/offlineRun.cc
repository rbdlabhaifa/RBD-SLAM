
#include "path_builder.hpp"

#include <geos/geom/GeometryFactory.h>
#include <lemon/core.h>
#include <lemon/list_graph.h>
#include <lemon/path.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <numeric>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>
#include <pcl/impl/point_types.hpp>
#include "auxilary.hpp"
#include "eigen_operations.hpp"
#include "pcl_operations.hpp"
#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>

#include <opencv2/core.hpp>
using namespace Auxilary;
using namespace PCLOperations;
using namespace EigenOperations;

std::vector<std::vector<pcl::PointXYZ>> PathBuilder::get_all_paths_to_leaves(
    const lemon::ListDigraph& graph,
    const lemon::ListDigraph::NodeMap<pcl::PointXYZ>& node_map,
    const lemon::ListDigraph::Node& start_node) {
    std::vector<std::vector<pcl::PointXYZ>> paths;

    std::size_t amount = 0;

    lemon::Dfs<lemon::ListDigraph> dfs(graph);
    dfs.init();
    dfs.addSource(start_node);
    dfs.start();

    std::size_t leaves = 0;

    for (lemon::ListDigraph::NodeIt it(graph); it != lemon::INVALID; ++it) {
        if (lemon::countOutArcs(graph, it) != 0) {
            continue;
        }

        ++leaves;

        std::vector<pcl::PointXYZ> current_path;
        current_path.insert(current_path.begin(), node_map[it]);

        auto prev = dfs.predNode(it);

        while (prev != lemon::INVALID) {
            current_path.insert(current_path.begin(), node_map[prev]);
            prev = dfs.predNode(prev);
        }

        paths.push_back(current_path);
    }

    std::cout << leaves << " LEAVES from " << lemon::countNodes(graph)
              << std::endl;

    return paths;
}

int main(){
    
    return 0;
}