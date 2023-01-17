cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(PointCloudFindOrbSlam)
set(CMAKE_EXPORT_COMPILE_COMMANDS
    ON
    CACHE INTERNAL "") # works
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(external/ORB_SLAM2)

find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)
find_package(LEMON REQUIRED)
find_package(OpenCV 4.0 REQUIRED)
find_package(spdlog REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Tello + Stream
add_library(tello_stream src/streamer.cc src/drone.cc)
target_include_directories(tello_stream PUBLIC include)
target_link_libraries(tello_stream PRIVATE ${OpenCV_LIBS} ${Boost_LIBRARIES}
                                           ctello)

# Path Builder
add_library(path_builder src/path_builder.cc src/auxilary.cc)
target_include_directories(path_builder PUBLIC include)
target_include_directories(path_builder PRIVATE ${LEMON_INCLUDE_DIR})
target_link_libraries(path_builder PRIVATE ${LEMON_LIBRARIES} ${PCL_LIBRARIES})

# Auto Navigation App
add_executable(
  auto_navigation src/apps/auto_navigation.cc src/navigator.cc src/explorer.cc
                  src/path_builder.cc src/auxilary.cc)
target_link_libraries(auto_navigation PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen)
target_link_libraries(auto_navigation PRIVATE ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(auto_navigation PRIVATE ORB_SLAM2)
target_link_libraries(auto_navigation PRIVATE tello_stream path_builder)
target_include_directories(auto_navigation PRIVATE include)

# Map Builder App
add_executable(map_builder src/utils/map_builder.cc)
target_link_libraries(map_builder PRIVATE ORB_SLAM2 ${OpenCV_LIBS}
                                          ${Boost_LIBRARIES} tello_stream)
target_include_directories(map_builder PRIVATE include)

add_executable(show_rrt src/utils/show_rrt.cc)
target_link_libraries(show_rrt PRIVATE ${PCL_LIBRARIES} path_builder)
target_include_directories(show_rrt PRIVATE include)
