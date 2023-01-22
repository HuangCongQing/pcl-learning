cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

# 1 添加pcl
# find_package(PCL REQUIRED)
set(PCL_DIR "/home/hcq/software_ubuntu/pcl-1.11_compile/share/pcl-1.11") # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,
find_package(PCL 1.11 REQUIRED COMPONENTS) #
# find_package(PCL  REQUIRED COMPONENTS) #
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# 2 添加opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${OpenCV_LIBRARIES}
#   ${PCL_INCLUDE_DIRS}
# )


# 3 添加boost thread多线程  
# set(BOOST_LIBRARYDIR /usr/lib)
# find_package(Boost REQUIRED COMPONENTS thread)

# set(THREADS_PREFER_PTHREAD_FLAG ON)
# find_package(Threads REQUIRED)

# fix: undefined reference to symbol 'pthread_condattr_setclock@@GLIBC_2.3.3'
# set(CMAKE_CXX_FLAGS -pthread)
# message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}") # -- CMAKE_CXX_FLAGS = -pthread

# 修改路径
add_executable (main main.cpp KMeans.cpp)
target_link_libraries (main 
                        ${PCL_LIBRARIES}   # PCL
                        ${OpenCV_LIBRARIES} # opencv
                    )