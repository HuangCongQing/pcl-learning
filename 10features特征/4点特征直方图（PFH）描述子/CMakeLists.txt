cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PFH PFH.cpp)
target_link_libraries (PFH ${PCL_LIBRARIES})
add_executable (PFH1 PFH1.cpp)
target_link_libraries (PFH1 ${PCL_LIBRARIES})