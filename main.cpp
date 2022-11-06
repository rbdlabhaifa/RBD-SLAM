#include "lemon/core.h"
#include <chrono>
#include <iostream>
#include <lemon/bfs.h>
#include <lemon/list_graph.h>
#include <lemon/maps.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <random>
#include <sstream>
#include <thread>

float ScaleFactor = 0.5;
using namespace std::chrono_literals;

struct Edge {
  pcl::PointXYZ p_point1;
  pcl::PointXYZ p_point2;
};

/**
 * @brief This function search for additional points in the radius
 * of "PointXYZ searchPoint"
 * @param cloud -> The Cloud
 * @param searchPoint -> the point I am searching around
 * @param radius -> The radius of search
 * @return -> number of points that are in the neigbourhood of searchPoint
 */
int radiusSearch(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                 pcl::PointXYZ searchPoint, float radius,
                 pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree) {
  // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  // kdtree.setInputCloud(cloud);
  //  Neighbors within radius search
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  return kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                             pointRadiusSquaredDistance);
}
pcl::PointXYZ operator-(pcl::PointXYZ p1, pcl::PointXYZ p2) {
  pcl::PointXYZ p3;
  p3.x = p1.x - p2.x;
  p3.y = p1.y - p2.y;
  p3.z = p1.z - p2.z;
  return p3;
}
float norm(pcl::PointXYZ p1) {
  return sqrt(p1.x * p1.x + p1.y * p1.y + p1.z * p1.z);
}
pcl::PointXYZ operator/(pcl::PointXYZ p1, float d) {
  pcl::PointXYZ p3;
  if (d == 0) {
    p3.x = p1.x;
    p3.y = p1.y;
    p3.z = p1.z;
    return p3;
  }
  p3.x = p1.x / d;
  p3.y = p1.y / d;
  p3.z = p1.z / d;
  return p3;
}
/**
 * @brief copy point1 to point2
 * @param pcl::PointXYZ point1[in] -> input point
 * @param pcl::PointXYZ point2[in] -> output
 * @return -> None
 */
void copy(pcl::PointXYZ &point1, pcl::PointXYZ &point2) {
  point2.x = point1.x;
  point2.y = point1.y;
  point2.z = point1.z;
}
/**
 * @brief Given 2 vectors of size 3*1 and 3*1
 * returns the matrix multipication
 * @param pcl::PointXYZ p1[in] -> is the first point
 * @param pcl::PointXYZ p2[in] -> is the second point
 * @return float -> The matrix mul (1*3)*(3*1)
 */
float operator*(pcl::PointXYZ p1, pcl::PointXYZ p2) {
  return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}
pcl::PointXYZ operator*(pcl::PointXYZ p1, float a) {
  return pcl::PointXYZ(p1.x * a, p1.y * a, p1.z * a);
}

pcl::PointXYZ operator*(float a, pcl::PointXYZ p1) {
  return pcl::PointXYZ(p1.x * a, p1.y * a, p1.z * a);
}
pcl::PointXYZ operator+(pcl::PointXYZ p1, pcl::PointXYZ p2) {
  return pcl::PointXYZ(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
}

/**
 * @brief
 * This function build a line between @var start and @var end
 * and return all inside the line with distance at least
 * @var jump_distance
 * @param pcl::PointXYZ start -> 1 of the points to create a line
 * @param pcl::PointXYZ end -> the other point to create the line
 * @param float jump_distance -> sets the minimum distance of the point on line
 * @return -> all points on line with distance at least @var jump_distance
 */
std::vector<pcl::PointXYZ> get_points_on_line(pcl::PointXYZ start,
                                              pcl::PointXYZ end,
                                              float jump_distance) {
  std::vector<pcl::PointXYZ> points_on_line;
  pcl::PointXYZ start2end;
  start2end = end - start;
  float total_travel_line = sqrt(start2end * start2end);
  pcl::PointXYZ hat_p = start2end / total_travel_line;
  for (float i = jump_distance * 1; i < total_travel_line;
       i = i + jump_distance) {
    pcl::PointXYZ p = start + hat_p * i;
    // std::cout << p.x << "," << p.y <<"," << p.z <<std::endl;
    points_on_line.push_back(p);
  }
  return points_on_line;
}

bool is_valid_movement(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                       pcl::PointXYZ current_point, pcl::PointXYZ dest_point,
                       pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree) {
  float jump_distance = 0.1 * ScaleFactor; // Magic number ?
  float radius_search = 0.25 * ScaleFactor;
  std::vector<pcl::PointXYZ> v_points_on_line =
      get_points_on_line(current_point, dest_point, jump_distance);
  for (pcl::PointXYZ &p : v_points_on_line) {
    if (radiusSearch(cloud, p, radius_search, kdtree) > 5) {
      // std::cout <<"Found Obstacle"<< p.x << "," <<p.y << "," <<p.z<<
      // std::endl;
      return false;
    }
  }
  return true;
}
// void test_valid_movement(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
//                          pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {
//   pcl::PointXYZ p1 = {-5, 0.5, 3};
//   pcl::PointXYZ p2 = {-5, 0.5, 4};
//   if (is_valid_movement(cloud, p1, p2, kdtree) == true) {
//     std::cout << " Success ! " << std::endl;
//   } else {
//     std::cout << " Failed  " << std::endl;
//   }
// }

/**
 * @brief This function finds random point on plane
 * the plane is define by those three points which are point1 point2 point3
 * @warning point1 , point2 , point3 should not be on the same line
 * preferably they should be perpedicular
 * @param point1[in] -> should be point on plane
 * @param point2[in] -> should be point on plane
 * @param point3[in] -> should be point on plane
 * @returns randomVectorOnPlane -> random point on plane !
 *
 * */
pcl::PointXYZ getRandomPointOnPlaneDefBy3Points(pcl::PointXYZ point1,
                                                pcl::PointXYZ point2,
                                                pcl::PointXYZ point3) {
  pcl::PointXYZ spanVector1 = point3 - point1;
  pcl::PointXYZ spanVector2 = point3 - point2;

  std::random_device
      rd; // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(-30.0, 30.0);
  float RandomNum1 = dis(gen);
  float RandomNum2 = dis(gen);
  pcl::PointXYZ randomVectorOnPlane =
      RandomNum1 * spanVector1 + RandomNum2 * spanVector2 + point1;
  return randomVectorOnPlane;
}

pcl::visualization::PCLVisualizer::Ptr
shapesVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
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
  //  viewer->addCoordinateSystem(20.0);
  // viewer->initCameraParameters ();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  //  viewer->addLine<pcl::PointXYZ> ((*cloud)[0],
  //                                     (*cloud)[cloud->size() - 1], "line");
  //  viewer->addSphere ((*cloud)[0], 0.2, 0.5, 0.5, 0.0, "sphere");
  // pcl::PointXYZ point1;
  // point1.x = -5;
  // point1.y = 0.5;
  // point1.z = 3;
  // viewer->addCoordinateSystem(1.0, point1.x, point1.y, point1.z);
  // viewer->addSphere(point1, 0.2, 0.5, 0.5, 0.0, "sphere1");
  // pcl::PointXYZ point2;
  // point2.x = -5;
  // point2.y = 0.5;
  // point2.z = 4;
  // viewer->addCoordinateSystem(1.0, point2.x, point2.y, point2.z);
  // viewer->addSphere(point2, 0.2, 0.5, 0.5, 0.0, "sphere2");
  // pcl::PointXYZ point3;
  // point3.x = -5;
  // point3.y = 0.5;
  // point3.z = 7;
  // viewer->addCoordinateSystem(1.0, point3.x, point3.y, point3.z);
  // viewer->addSphere(point3, 0.2, 1.0, 0.1, 0.1, "sphere3");
  return (viewer);
}

// Hashable function for pcl::PointXYZ
bool operator==(const pcl::PointXYZ &lhs, const pcl::PointXYZ &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && rhs.z == lhs.z;
}
struct MyHash {
  std::size_t operator()(pcl::PointXYZ const &s) const noexcept {
    std::size_t h1 = std::hash<float>{}(s.x);
    std::size_t h2 = std::hash<float>{}(s.y);
    std::size_t h3 = std::hash<float>{}(s.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2); // or use boost::hash_combine
  }
};
template <> struct std::hash<pcl::PointXYZ> {
  std::size_t operator()(pcl::PointXYZ const &s) const noexcept {
    std::size_t h1 = std::hash<float>{}(s.x);
    std::size_t h2 = std::hash<float>{}(s.y);
    std::size_t h3 = std::hash<float>{}(s.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2); // or use boost::hash_combine
  }
};

// end Hashable

int main(int argc, char **argv) {
  lemon::ListGraph RRTGraph;
  lemon::ListGraph::NodeMap<pcl::PointXYZ> nodeToPoint(RRTGraph);
  // std::unordered_map<pcl::PointXYZ, lemon::ListGraph::Node> pointToNode;
  // points2.pcd
  // pcl::PointXYZ knownPoint1 = {-0.264355 + 0.4, -0.105879 + 0.1, 0.166656};
  // pcl::PointXYZ knownPoint2 = {-0.281147, -0.032053 + 0.1, -0.252866};
  // pcl::PointXYZ knownPoint3 = {-0.661326, -0.038876 + 0.1, -0.242295};
  // points3.pcd
  // pcl::PointXYZ knownPoint1 = {-0.152039, -0.112927 , 0.521806};
  // pcl::PointXYZ knownPoint2 = {-0.0388202, 0.0144199 , -0.150134};
  // pcl::PointXYZ knownPoint3 = {0.0739677 , 0.0088157 , -0.170298};
  pcl::PointXYZ knownPoint1 = {0.114433, -0.0792644 + 0.0, 0.238077};
  pcl::PointXYZ knownPoint2 = {0.0119525, -0.00770169 + 0.0, -0.0757213};
  pcl::PointXYZ knownPoint3 = {0.548205, 0.0233536 + 0.0, -0.146354};

  //  2.98735 -0.312701 0.234717
  // 2.15991 -0.303712 0.337741
  // 3.11331 -0.574892 0.515388

  // pcl::PointXYZ knownPoint1 = {2.98735, -0.312701, 0.234717};
  // pcl::PointXYZ knownPoint3 = {2.15991, -0.303712, 0.337741};
  // pcl::PointXYZ knownPoint2 = {3.11331, -0.574892, 0.515388};

  if (argc < 2) {
    std::cout << "Please Enter pcd you want to analysis  e.g yam_data.pcd"
              << std::endl;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) ==
      -1) //* load the file
  {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  pcl::visualization::PCLVisualizer::Ptr viewer = shapesVis(cloud);
  // test RRT
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  pcl::PointXYZ navigate_starting_point = {0.114433, -0.0792644 + 0.0,
                                           0.238077};
  // navigate_starting_point.x = knownPoint1.x;
  // navigate_starting_point.y = knownPoint1.y;
  // navigate_starting_point.z = knownPoint1.z;
  std::vector<Edge> v_edges;
  pcl::PointCloud<pcl::PointXYZ>::Ptr navigate_data1(
      new pcl::PointCloud<pcl::PointXYZ>);
  navigate_data1->push_back(navigate_starting_point);
  lemon::ListGraph::Node firstNode = RRTGraph.addNode();
  nodeToPoint[firstNode] = navigate_starting_point;

  //  std::random_device
  //      rd; // Will be used to obtain a seed for the random number engine
  //  std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with
  //  rd() std::uniform_real_distribution<> dis(-30.0, 30.0); float jump = 0.5;

  float jump = 0.8 * ScaleFactor;
  for (int i = 0; i < 2000; i++) {
    pcl::PointXYZ point_rand = getRandomPointOnPlaneDefBy3Points(
        knownPoint1, knownPoint2, knownPoint3);
    pcl::KdTreeFLANN<pcl::PointXYZ> navigate_tree1;
    navigate_tree1.setInputCloud(navigate_data1);
    std::vector<int> v_closest_point1(1);
    std::vector<float> v_dist_closest_point1(1);
    if (navigate_tree1.nearestKSearch(point_rand, 1, v_closest_point1,
                                      v_dist_closest_point1)) {
      pcl::PointXYZ point_new =
          (*navigate_data1)[v_closest_point1[0]] +
          jump * (point_rand - (*navigate_data1)[v_closest_point1[0]]) /
              (sqrt((point_rand - (*navigate_data1)[v_closest_point1[0]]) *
                    (point_rand - (*navigate_data1)[v_closest_point1[0]])));
      if (is_valid_movement(cloud, (*navigate_data1)[v_closest_point1[0]],
                            point_new, kdtree)) {
        navigate_data1->push_back(point_new);
        Edge e = {point_new, (*navigate_data1)[v_closest_point1[0]]};
        v_edges.push_back(e);

        lemon::ListGraph::Node newNode = RRTGraph.addNode();
        // pointToNode[point_new] = newNode;
        nodeToPoint[newNode] = point_new;

        lemon::ListGraph::Node closestPointInRRT;
        for (lemon::ListGraph::NodeIt it(RRTGraph); it != lemon::INVALID;
             ++it) {
          if (nodeToPoint[it] == (*navigate_data1)[v_closest_point1[0]]) {
            closestPointInRRT = it;
          }
        }
        lemon::ListGraph::Edge newEdge =
            RRTGraph.addEdge(newNode, closestPointInRRT);
      }
    } else {
      std::cout << "cannot find nearest Node!!" << std::endl;
    }
  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  int index = 1;
  for (Edge edge : v_edges) {
    std::stringstream ss;
    ss << "navigate_line" << index;
    viewer->addLine<pcl::PointXYZ>((edge.p_point1), (edge.p_point2), ss.str());
    index++;
  }
  index = 1;
  for (pcl::PointXYZ p : *navigate_data1) {
    std::stringstream ss;
    ss << "PointNavigate" << index;
    viewer->addSphere(p, 0.2 * ScaleFactor, 0.9, 0.2, 0.0, ss.str());
    index++;
  }

  std::cout << v_edges.size() << std::endl;
  std::cout << (*navigate_data1).size() << std::endl;
  for (pcl::PointXYZ p : *navigate_data1) {
    // std::cout << p.x << "," << p.y << "," << p.z;
  }

  // Random Vertices from the RRTGraph!
  std::random_device os_seed;
  const uint_least32_t seed = os_seed();

  std::mt19937 generator(seed);
  std::uniform_int_distribution<uint_least32_t> distribute(
      1, lemon::countNodes(RRTGraph) - 2);
  lemon::ListGraph::NodeIt n(RRTGraph);
  for (int i = 0; i < distribute(generator); i++) {
    ++n;
  }
  lemon::ListGraph::Node get_random_node_from_RRT = n;
  // End Random Vertices from the RRTGraph!

  // BFS
  lemon::Bfs<lemon::ListGraph> bfs(RRTGraph);
  bfs.init();
  bfs.addSource(firstNode);
  bfs.start();

  if (bfs.reached(get_random_node_from_RRT)) {
    std::cout << nodeToPoint[get_random_node_from_RRT];
    lemon::ListGraph::Node prev = bfs.predNode(get_random_node_from_RRT);
    while (prev != lemon::INVALID) {
      std::cout << "<-" << nodeToPoint[prev];
      prev = bfs.predNode(prev);

      std::stringstream ss;
      ss << "PointNavigatePath" << index;
      viewer->addSphere(nodeToPoint[prev], 0.2 * ScaleFactor, 0.1, 0.2, 0.9,
                        ss.str());
      index++;
    }
    std::cout << std::endl;
  }

  viewer->addCoordinateSystem(1.0, navigate_starting_point.x,
                              navigate_starting_point.y,
                              navigate_starting_point.z);

  // pcl::PointXYZ p_helper{0.0867836,0.031304,0.0313494};
  //  pcl::PointXYZ p_helper{0.152039, -0.112927,0.521806};
  // pcl::PointXYZ p_helper{-0.804435 , -0.00937385 , 0.289798};
  // Print Path
  // viewer->addSphere(pcl::PointXYZ(-0.804435, -0.00937385, 0.289798),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 1.0, "SHPERE121");
  // viewer->addSphere(pcl::PointXYZ(-0.684641, -0.0372174, 0.37567),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.9, "SHPERE122");
  // viewer->addSphere(pcl::PointXYZ(-0.568991, -0.0657816, 0.466824),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.8, "SHPERE123");
  // viewer->addSphere(pcl::PointXYZ(-0.460346, -0.0953637, 0.565926),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.7, "SHPERE124");
  // viewer->addSphere(pcl::PointXYZ(-0.342639, -0.123583, 0.65452),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.6, "SHPERE125");
  // viewer->addSphere(pcl::PointXYZ(-0.193642, -0.138288, 0.663672),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.5, "SHPERE126");
  // viewer->addSphere(pcl::PointXYZ(-0.152039, -0.112927, 0.521806),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.4, "SHPERE127");
  // viewer->addSphere(pcl::PointXYZ(0, 0, 0), 0.15 * ScaleFactor, 0.3, 0.5,
  // 0.5,
  //                   "SHPERE127asdfasDolevf");

  pcl::PointXYZ knownPoint4 = {-0.264355 + 0.6, -0.105879 + 0.1, 0.166656};
  viewer->addSphere(pcl::PointXYZ(0.320001, 0.028691, -0.409289),
                    0.3 * ScaleFactor, 0.15, 0.5, 0.5, "SHPERE127asdfasDolevf");
  viewer->addSphere(knownPoint4, 0.15 * ScaleFactor, 0.3, 0.5, 0.5,
                    "SHPERE127asdfasDolevf1");

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
}
