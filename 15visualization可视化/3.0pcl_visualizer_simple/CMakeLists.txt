cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (1pcl_visualizer_simple 1pcl_visualizer_simple.cpp)
target_link_libraries (1pcl_visualizer_simple ${PCL_LIBRARIES})