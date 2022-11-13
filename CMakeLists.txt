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
find_package(OpenCV 3.0 REQUIRED)

# add_executable (example main.cpp) target_link_libraries (example
# Eigen3::Eigen)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(visualize main.cpp src/Streamer.cpp src/Navigator.cpp
                         src/Drone.cpp)
target_link_libraries(visualize PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen)
target_link_libraries(visualize PRIVATE ${LEMON_LIBRARIES})
target_link_libraries(visualize PRIVATE ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(visualize PRIVATE ORB_SLAM2 ctello)
target_include_directories(visualize PRIVATE include)
target_include_directories(visualize PRIVATE ${LEMON_INCLUDE_DIR})
