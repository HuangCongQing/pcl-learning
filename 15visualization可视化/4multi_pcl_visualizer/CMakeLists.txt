cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (multi_pcl_visualizer multi_pcl_visualizer.cpp)
target_link_libraries (multi_pcl_visualizer ${PCL_LIBRARIES})