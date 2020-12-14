cmake_minimum_required(VERSION 2.8.3)
project(argo_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	pcl_ros
	roscpp
	sensor_msgs
	std_msgs
	pcl_conversions
	rospy
	tf
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS 
	pcl_ros
	roscpp
	sensor_msgs
	pcl_conversions
)

include_directories(
	include
	src
	${catkin_INCLUDE_DIRS}
)
 

add_executable(
	viz_track_result
	src/viz_track_result.cpp)
target_link_libraries(
	viz_track_result
	${catkin_LIBRARIES}
	json
)
