cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${OpenCV_LIBRARIES}
#   ${PCL_INCLUDE_DIRS}
# )


add_executable (01PointXYZlidar2image 01PointXYZlidar2image.cpp)
target_link_libraries (01PointXYZlidar2image ${PCL_LIBRARIES}  ${OpenCV_LIBRARIES})

# add_executable (02PointXYZRGBlidar2image 02PointXYZRGBlidar2image.cpp)
# target_link_libraries (02PointXYZRGBlidar2image ${PCL_LIBRARIES}  ${OpenCV_LIBRARIES})