



cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(planar_segmentation)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "") # works


find_package(PCL 1.2 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( Boost REQUIRED )
find_package(LEMON REQUIRED)




#add_executable (example main.cpp)
#target_link_libraries (example Eigen3::Eigen)



include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (visualize main.cpp)
target_link_libraries (visualize PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen)
target_link_libraries(visualize PRIVATE ${LEMON_LIBRARIES})
target_include_directories(visualize PRIVATE ${LEMON_INCLUDE_DIR})
