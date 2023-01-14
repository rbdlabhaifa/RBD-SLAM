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

add_library(tello_stream src/streamer.cc src/drone.cc)
target_include_directories(tello_stream PUBLIC include)
target_link_libraries(tello_stream PRIVATE ${OpenCV_LIBS} ${Boost_LIBRARIES}
                                           ctello)

add_executable(visualize src/main2.cpp src/navigator.cc src/explorer.cc)
target_link_libraries(visualize PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen)
target_link_libraries(visualize PRIVATE ${LEMON_LIBRARIES} ${OpenCV_LIBS}
                                        ${Boost_LIBRARIES})
target_link_libraries(visualize PRIVATE ORB_SLAM2)
target_link_libraries(visualize PRIVATE tello_stream)
target_include_directories(visualize PRIVATE include)
target_include_directories(visualize PRIVATE ${LEMON_INCLUDE_DIR})

add_executable(map_builder src/utils/map_builder.cc)
target_link_libraries(map_builder PRIVATE ORB_SLAM2 ${OpenCV_LIBS}
                                          ${Boost_LIBRARIES} tello_stream)
target_include_directories(map_builder PRIVATE include)
