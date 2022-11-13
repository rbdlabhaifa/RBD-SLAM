
cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -lboost_serialization")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall   -O3 -lboost_serialization -march=native")


include(CheckCXXCompilerFlag)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
add_definitions(-DCOMPILEDWITHC17)
message(STATUS "Using flag -std=c++17.")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif ()
endif ()

find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization)

#include_directories(
#        ${PROJECT_SOURCE_DIR}
#        ${PROJECT_SOURCE_DIR}/include
#        ${Pangolin_INCLUDE_DIRS}
#        ${Boost_INCLUDE_DIRS}
#
#)

SET(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3/")
IF (NOT EIGEN3_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF ()
INCLUDE_DIRECTORIES("${EIGEN3_INCLUDE_DIR}")

add_subdirectory(Thirdparty)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(ORB_SLAM2 SHARED
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        src/RaspberryKeyFrame.cc
        src/CSVReader.cc
        )

target_link_libraries(ORB_SLAM2
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${Boost_LIBRARIES}
	g2o
	DBoW2
        #${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        #${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        )
target_include_directories(ORB_SLAM2 PRIVATE         
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${Pangolin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
) 
 target_include_directories(ORB_SLAM2 INTERFACE        
 	${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
 )


