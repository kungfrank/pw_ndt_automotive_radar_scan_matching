cmake_minimum_required(VERSION 2.8.3)
project(radar_odometry)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

set (OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
find_package( OpenCV REQUIRED )

#find_package(PCL 1.3 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  conti_radar
  cv_bridge
  ro_msg
  #OpenCV
  ndt_registration
  #ndt_map
  #ndt_generic

  sensor_msgs
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES radar_odometry
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

#=== RO Cen 1 ===#
add_executable(da src/discriptor_da.cpp)
target_link_libraries(da ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

##temp##
add_executable(compress_read src/compress_read.cpp)
target_link_libraries(compress_read ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#=== Instantaneous ego-motion estimation using multiple Doppler radars ===#
add_executable(radar_ego_motion src/radar_ego_motion.cpp)
target_link_libraries(radar_ego_motion ${catkin_LIBRARIES})

#add_executable(radar_ego_motion_ack src/radar_ego_motion_ack.cpp)
#target_link_libraries(radar_ego_motion_ack ${catkin_LIBRARIES})
#add_executable(radar_ego_motion_valid src/radar_ego_motion_valid.cpp)
#target_link_libraries(radar_ego_motion_valid ${catkin_LIBRARIES})
#add_executable(radar_ego_motion_valid2 src/radar_ego_motion_valid2.cpp)
#target_link_libraries(radar_ego_motion_valid2 ${catkin_LIBRARIES})
#add_executable(radar_ego_motion_valid3 src/radar_ego_motion_valid3.cpp)
#target_link_libraries(radar_ego_motion_valid3 ${catkin_LIBRARIES})
#add_executable(radar_ego_motion_self_sync src/radar_ego_motion_self_sync.cpp)
#target_link_libraries(radar_ego_motion_self_sync ${catkin_LIBRARIES})

add_executable(vel_intgration src/vel_intgration.cpp)
target_link_libraries(vel_intgration ${catkin_LIBRARIES})


add_executable(odom_error_cal src/odom_error_cal.cpp)
target_link_libraries(odom_error_cal ${catkin_LIBRARIES})
add_executable(odom_error_cal_tf_version src/odom_error_cal_tf_version.cpp)
target_link_libraries(odom_error_cal_tf_version ${catkin_LIBRARIES})
add_executable(odom_error_cal_kitti src/odom_error_cal_kitti.cpp)
target_link_libraries(odom_error_cal_kitti ${catkin_LIBRARIES})

add_executable(error_mean src/error_mean.cpp)
target_link_libraries(error_mean ${catkin_LIBRARIES})
add_executable(error_mean_kitti src/error_mean_kitti.cpp)
target_link_libraries(error_mean_kitti ${catkin_LIBRARIES})

add_executable(kitti_odom_file_generator src/kitti_odom_file_generator.cpp)
target_link_libraries(kitti_odom_file_generator ${catkin_LIBRARIES})
add_executable(kitti_odom_file_generator_tf_version src/kitti_odom_file_generator_tf_version.cpp)
target_link_libraries(kitti_odom_file_generator_tf_version ${catkin_LIBRARIES})
add_executable(kitti_odom_file_generator_single src/kitti_odom_file_generator_single.cpp)
target_link_libraries(kitti_odom_file_generator_single ${catkin_LIBRARIES})


add_executable(conti_merge_td_comp src/conti_merge_td_comp.cpp)
target_link_libraries(conti_merge_td_comp ${catkin_LIBRARIES})
add_executable(conti_stack src/conti_stack.cpp)
target_link_libraries(conti_stack ${catkin_LIBRARIES})
add_executable(conti_stack_cov src/conti_stack_cov.cpp)
target_link_libraries(conti_stack_cov ${catkin_LIBRARIES})
add_executable(conti_stack_cov_with_p_unc src/conti_stack_cov_with_p_unc.cpp)
target_link_libraries(conti_stack_cov_with_p_unc ${catkin_LIBRARIES})

add_executable(conti_stack_debug_publisher src/conti_stack_debug_publisher.cpp)
target_link_libraries(conti_stack_debug_publisher ${catkin_LIBRARIES})
add_executable(downsampled_semi_dense src/downsampled_semi_dense.cpp)
target_link_libraries(downsampled_semi_dense ${catkin_LIBRARIES})


add_executable(stack_icp src/stack_icp.cpp)
target_link_libraries(stack_icp ${catkin_LIBRARIES})
add_executable(stack_ndt src/stack_ndt.cpp)
target_link_libraries(stack_ndt ${catkin_LIBRARIES})
add_executable(stack_ndt_d2d_2d src/stack_ndt_d2d_2d.cpp)
target_link_libraries(stack_ndt_d2d_2d ${catkin_LIBRARIES})
add_executable(stack_gnd_costmap src/stack_gnd_costmap.cpp)
target_link_libraries(stack_gnd_costmap ${catkin_LIBRARIES})


add_executable(radar_scan_to_img src/radar_scan_to_img.cpp)
target_link_libraries(radar_scan_to_img ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#=== LO ===#
add_executable(lidar_preprocess src/lo/lidar_preprocess.cpp)
target_link_libraries(lidar_preprocess ${catkin_LIBRARIES})
add_executable(lidar_odometry src/lo/lidar_odometry.cpp)
target_link_libraries(lidar_odometry ${catkin_LIBRARIES})

#=== Probabilistic ego-motion estimation using multiple automotive radar sensors ===#
add_executable(conti_joint_doppler_weight src/joint_doppler_ndt/conti_joint_doppler_weight.cpp)
target_link_libraries(conti_joint_doppler_weight ${catkin_LIBRARIES})
add_executable(joint_doppler_ndt src/joint_doppler_ndt/joint_doppler_ndt.cpp)
target_link_libraries(joint_doppler_ndt ${catkin_LIBRARIES})




add_executable(ndt_test src/ndt_test.cpp)
target_link_libraries(ndt_test ${catkin_LIBRARIES})

find_package(CUDA REQUIRED)

find_package(ndt_gpu REQUIRED)
include_directories(${ndt_gpu_INCLUDE_DIRS})
link_directories(${ndt_gpu_INCLUDE_DIRS})

add_executable(ndt_gpu_test src/ndt_gpu_test.cpp)
target_link_libraries(ndt_gpu_test ${catkin_LIBRARIES} ${ndt_gpu_LIBRARIES})
target_include_directories(ndt_gpu_test PRIVATE ${CUDA_INCLUDE_DIRS})

###########################################################################

find_package(CUDA REQUIRED)

set(CUDA_ARCH "sm_52")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++11;--ptxas-options=-v)

CUDA_ADD_EXECUTABLE(cuda_test src/cuda_test.cu)
target_link_libraries(cuda_test ${catkin_LIBRARIES})






