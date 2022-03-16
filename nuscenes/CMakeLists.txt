cmake_minimum_required(VERSION 2.8.3)
project(nuscenes)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
	conti_radar
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(conti_merge src/conti_merge.cpp)
target_link_libraries(conti_merge ${catkin_LIBRARIES})

add_executable(conti_merge_rcs src/conti_merge_rcs.cpp)
target_link_libraries(conti_merge_rcs ${catkin_LIBRARIES})

add_executable(radar_viz src/radar_viz.cpp)
target_link_libraries(radar_viz ${catkin_LIBRARIES})

add_executable(radar_viz_2 src/radar_viz_2.cpp)
target_link_libraries(radar_viz_2 ${catkin_LIBRARIES})

add_executable(radar_viz_3 src/radar_viz_3.cpp)
target_link_libraries(radar_viz_3 ${catkin_LIBRARIES})

add_executable(radar_vel_viz src/radar_vel_viz.cpp)
target_link_libraries(radar_vel_viz ${catkin_LIBRARIES})

add_executable(origin_tf_publisher src/origin_tf_publisher.cpp)
target_link_libraries(origin_tf_publisher ${catkin_LIBRARIES})

add_executable(gt_vel_publisher src/gt_vel_publisher.cpp)
target_link_libraries(gt_vel_publisher ${catkin_LIBRARIES})

add_executable(gt_odom_publisher src/gt_odom_publisher.cpp)
target_link_libraries(gt_odom_publisher ${catkin_LIBRARIES})


