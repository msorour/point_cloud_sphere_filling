cmake_minimum_required(VERSION 2.8.3)

set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

find_package(PCL 1.7 REQUIRED COMPONENTS)
#find_package(VTK REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${rospy_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${roscpp_LIBRARY_DIRS})
link_directories(${rospy_LIBRARY_DIRS})
link_directories(${std_msgs_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

#set (CMAKE_CXX_FLAGS "-O2 -Wall")
set (CMAKE_CXX_STANDARD 11)

add_executable(point_cloud_as_a_set_of_spheres point_cloud_as_a_set_of_spheres.cpp)
target_link_libraries(point_cloud_as_a_set_of_spheres pthread ${PCL_LIBRARIES})

