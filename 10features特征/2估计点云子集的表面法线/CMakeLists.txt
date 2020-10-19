cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (normal_estimation_subdset_points normal_estimation_subdset_points.cpp)
target_link_libraries (normal_estimation_subdset_points ${PCL_LIBRARIES})