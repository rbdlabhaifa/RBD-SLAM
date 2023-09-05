cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(PointCloudFindOrbSlam)
set(CMAKE_EXPORT_COMPILE_COMMANDS
    ON
    CACHE INTERNAL "") # works
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_compile_definitions(USE_UNSTABLE_GEOS_CPP_API)

add_library(ORB_SLAM3 SHARED IMPORTED)
set_property(
  TARGET ORB_SLAM3
  PROPERTY IMPORTED_LOCATION
           ${CMAKE_SOURCE_DIR}/external/ORB_SLAM3/lib/libORB_SLAM3.so)

set(ORB_SLAM3_INCLUDES
    external/ORB_SLAM3/include external/ORB_SLAM3/include/CameraModels
    external/ORB_SLAM3 external/ORB_SLAM3/Thirdparty/Sophus
    external/ORB_SLAM3/Thirdparty/g2o external/ORB_SLAM3/Thirdparty/DBoW2)

find_package(PCL 1.2 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)
find_package(LEMON REQUIRED)
find_package(OpenCV 4.0 REQUIRED)
find_package(GEOS REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Tello + Stream
add_library(tello_stream src/streamer.cc src/drone.cc)
target_include_directories(tello_stream PRIVATE include)
target_link_libraries(tello_stream PRIVATE ${Boost_LIBRARIES} ctello
                                           ${OpenCV_LIBS})

# GOAL Finder
add_library(goal_finder src/goal_finder.cc)
target_include_directories(goal_finder PRIVATE include)

# PCL Operations
add_library(pcl_operations src/pcl_operations.cc)
target_include_directories(pcl_operations PRIVATE include)
target_link_libraries(pcl_operations PRIVATE ${PCL_LIBRARIES})

# Eigen Operations
add_library(eigen_operations src/eigen_operations.cc)
target_include_directories(eigen_operations PRIVATE include)
target_link_libraries(eigen_operations PRIVATE Eigen3::Eigen)

# Path Builder
add_library(path_builder src/path_builder.cc src/auxilary.cc)
target_include_directories(path_builder PRIVATE include)
target_include_directories(path_builder PRIVATE ${LEMON_INCLUDE_DIR})
target_link_libraries(
  path_builder PRIVATE ${LEMON_LIBRARIES} ${PCL_LIBRARIES} GEOS::geos
                       Eigen3::Eigen pcl_operations eigen_operations)
target_link_libraries(path_builder PUBLIC ${OpenCV_LIBS})

# SLAM Utils
add_library(slam_utils src/slam_utils.cc)
target_include_directories(slam_utils PRIVATE include ${ORB_SLAM3_INCLUDES})
target_link_libraries(slam_utils PRIVATE ORB_SLAM3 ${OpenCV_LIBS}
                                         ${Pangolin_LIBRARIES})


# Auto Navigation App
add_executable(auto_navigation src/apps/auto_navigation.cc src/navigator.cc
                               src/explorer.cc)
target_link_libraries(auto_navigation PRIVATE ${PCL_LIBRARIES})
target_link_libraries(auto_navigation PRIVATE ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(auto_navigation PRIVATE ORB_SLAM3 ${Pangolin_LIBRARIES})
target_link_libraries(auto_navigation PRIVATE tello_stream path_builder
                                              slam_utils goal_finder)
target_include_directories(auto_navigation PRIVATE include
                                                   ${ORB_SLAM3_INCLUDES})

# Map Builder App
add_executable(map_builder src/utils/map_builder.cc)
target_link_libraries(
  map_builder PRIVATE ORB_SLAM3 ${Pangolin_LIBRARIES} ${OpenCV_LIBS}
                      ${Boost_LIBRARIES} tello_stream slam_utils)
target_include_directories(map_builder PRIVATE include ${ORB_SLAM3_INCLUDES})

add_executable(show_rrt src/utils/show_rrt.cc src/explorer.cc)
target_link_libraries(show_rrt PRIVATE ${PCL_LIBRARIES} path_builder)
target_include_directories(show_rrt PRIVATE include)
