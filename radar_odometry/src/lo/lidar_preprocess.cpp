#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <vector>
#include <iostream>
#include <thread>
#include <pthread.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <chrono>
#include <fstream>
#include <sstream>
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> NoCloudSyncPolicy;

//-- param --//

ros::Publisher pc_pub;

pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
sensor_msgs::PointCloud2 ros_pc2;

///////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZ>::Ptr filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1.44, 10); //for icp (-1.44, 10)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter (*cloud_z_filtered);
  return cloud_z_filtered;
}


void Callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg){
  //cout << "=============================== \n";
  //cout.precision(10);
  //cout << "pc time: " << pc_msg->header.stamp.toSec() << endl;


  ///--- convert pointcloud msg to pc ---///
  pcl::PCLPointCloud2* pc2 = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*pc_msg, *pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*pc2, *pc);

  pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setMin(Eigen::Vector4f(-0.75, -1.5, -2.0,1.0));
  crop.setMax(Eigen::Vector4f(0.75, 2.5, 0.0,1.0));
  crop.setTranslation(Eigen::Vector3f(0.0, 0.0, 0.0));
  crop.setRotation(Eigen::Vector3f(0.0, 0.0, 0.0));
  crop.setInputCloud(pc);
  crop.setNegative(true);
  crop.filter(*crop_cloud);

  ///--- filter ground ---///
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  filtered_cloud = filterGround(crop_cloud);
  //filtered_cloud = crop_cloud;

  if (!filtered_cloud->is_dense)
  {
		filtered_cloud->is_dense = false;
		std::vector< int > indices;
		pcl::removeNaNFromPointCloud(*filtered_cloud,*filtered_cloud, indices);
  }

  ///--- publish combined pointcloud ---///
  pcl::toPCLPointCloud2(*filtered_cloud, *pc2);
  pcl_conversions::fromPCL(*pc2, ros_pc2);
  ros_pc2.is_dense = false;
  ros_pc2.header.frame_id = "/nuscenes_lidar";
  //ros_pc2.header.stamp = right_pc_msg->header.stamp;
  ros_pc2.header.stamp = pc_msg->header.stamp;
  pc_pub.publish(ros_pc2);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_odometry");
  ros::NodeHandle n;

  pc_pub = n.advertise<sensor_msgs::PointCloud2>("/nuscenes_lidar_new", 1);

	ros::Subscriber sub = n.subscribe("/nuscenes_lidar", 1000, Callback);

  ros::spin();

  return 0;
}
