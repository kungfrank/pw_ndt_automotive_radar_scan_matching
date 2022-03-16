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
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
//#include <tf/transform_broadcaster.h>
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
ros::Publisher lidar_odom_pub;

tf::Transform curr_tf;


pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_old (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered2_pc_old (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_old (new pcl::PointCloud<pcl::PointXYZ>);

bool init = false;

float old_pc_time;

///////////////////////////////////////////////

tf::Transform EigenToTf(Eigen::Matrix4d T){
  tf::Transform transform;

  tf::Vector3 origin;
  origin.setValue(static_cast<double>(T(0,3)),static_cast<double>(T(1,3)),static_cast<double>(T(2,3)));
  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(T(0,0)), static_cast<double>(T(0,1)), static_cast<double>(T(0,2)),
        static_cast<double>(T(1,0)), static_cast<double>(T(1,1)), static_cast<double>(T(1,2)),
        static_cast<double>(T(2,0)), static_cast<double>(T(2,1)), static_cast<double>(T(2,2)));

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  transform.setOrigin(origin);
  transform.setRotation(tfqt);

  return transform;
}

tf::Transform EigenToTf(Eigen::Matrix4f T){
  tf::Transform transform;

  tf::Vector3 origin;
  origin.setValue(static_cast<double>(T(0,3)),static_cast<double>(T(1,3)),static_cast<double>(T(2,3)));
  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(T(0,0)), static_cast<double>(T(0,1)), static_cast<double>(T(0,2)),
        static_cast<double>(T(1,0)), static_cast<double>(T(1,1)), static_cast<double>(T(1,2)),
        static_cast<double>(T(2,0)), static_cast<double>(T(2,1)), static_cast<double>(T(2,2)));

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  transform.setOrigin(origin);
  transform.setRotation(tfqt);

  return transform;
}

void Viz(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
         pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
         pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){

  // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);
  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  50000, "target cloud");
  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  50000, "output cloud");
  // Coloring and visualizing transformed input cloud (blue).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  input_color (source_cloud, 100, 100, 255);
  viewer_final->addPointCloud<pcl::PointXYZ> (source_cloud, input_color, "input cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  50000, "input cloud");
  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}


void Callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg){

//void Callback(const sensor_msgs::PointCloud2ConstPtr& radar_msg,
//							const sensor_msgs::PointCloud2ConstPtr& pc_msg){
  cout.precision(17);
  //cout << "radar_msg->header.stamp.toSec(): " << radar_msg->header.stamp.toSec() << endl;
  cout << "pc_msg->header.stamp.toSec(): " << pc_msg->header.stamp.toSec() << endl;

  //---- Convert to ro coordinate ----//
  sensor_msgs::PointCloud2 converted_pc_msg;
  Eigen::AngleAxisf z_rotation (-89.883/180*M_PI , Eigen::Vector3f::UnitZ ());
  Eigen::AngleAxisf y_rotation (0.338/180*M_PI , Eigen::Vector3f::UnitY ());
  Eigen::AngleAxisf x_rotation (-1.388/180*M_PI , Eigen::Vector3f::UnitX ());
  Eigen::Translation3f translation (0.944, 0, 1.84);
  Eigen::Matrix4f T = (translation * z_rotation * y_rotation * x_rotation).matrix ();
  pcl_ros::transformPointCloud(T, *pc_msg, converted_pc_msg);
  //---- End ----//

  pcl::PCLPointCloud2* pc2 = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(converted_pc_msg, *pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*pc2, *pc);

  ///--- filter pointcloud ---///
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (pc);
  approximate_voxel_filter.filter (*filtered_pc);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered2_pc (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter2;
  approximate_voxel_filter2.setLeafSize (0.05, 0.05, 0.05);
  approximate_voxel_filter2.setInputCloud (pc);
  approximate_voxel_filter2.filter (*filtered2_pc);


  // only do once
  if(init != true){
    init = true;
    filtered_pc_old = filtered_pc;
    filtered2_pc_old = filtered2_pc;
		pc_old = pc;
    old_pc_time = pc_msg->header.stamp.toSec();
    return;
  }

  Eigen::Matrix4f Tm(Eigen::Matrix4f::Identity());

  ///---------- ICP ----------///

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance (4.0); // 1m for 20hz, 4m for 4hz
  icp.setMaximumIterations (150);
  icp.setTransformationEpsilon(1e-6); //1e-6
  icp.setEuclideanFitnessEpsilon(1e-6); //1e-6
  icp.setInputSource(filtered_pc);
  icp.setInputTarget(filtered_pc_old);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*output_cloud); // no init guess
  Tm = icp.getFinalTransformation ();
  double score = icp.getFitnessScore (0.5);
  cout << "-------------------------------------" << endl;
  cout << " score: " << score << " ";

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
  icp2.setMaxCorrespondenceDistance (0.5);
  icp2.setMaximumIterations (200);
  icp2.setTransformationEpsilon(1e-8); //1e-6
  icp2.setEuclideanFitnessEpsilon(1e-8); //1e-6
  icp2.setInputSource(filtered_pc);
  icp2.setInputTarget(filtered_pc_old);
  icp2.align(*output_cloud, Tm);
  Tm = icp2.getFinalTransformation ();
  score = icp2.getFitnessScore (0.5);
  cout << " score: " << score << " ";

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp3;
  icp3.setMaxCorrespondenceDistance (0.25);
  icp3.setMaximumIterations (200);
  icp3.setTransformationEpsilon(1e-8);
  icp3.setEuclideanFitnessEpsilon(1e-8);
  icp3.setInputSource(pc);
  icp3.setInputTarget(pc_old);
  icp3.align(*output_cloud, Tm);
  Tm = icp3.getFinalTransformation ();
  score = icp3.getFitnessScore (0.25);
  cout << " score: " << score << endl;



  ///---------- NDT ----------///
/*
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setTransformationEpsilon (1e-2);
  ndt.setStepSize (0.1);
  ndt.setResolution (4.0);
  ndt.setMaximumIterations (100);
  ndt.setInputSource (filtered_pc);
  ndt.setInputTarget (filtered_pc_old);
  ndt.setOulierRatio(0.6);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud);
  Tm = ndt.getFinalTransformation ();
  cout << "=========================== \n";
  std::cout << " score: " << ndt.getFitnessScore (0.5) << std::endl;


  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt2;
  ndt2.setTransformationEpsilon (1e-6);
  ndt2.setStepSize (0.05); //0.005
  ndt2.setResolution (1.0);
  ndt2.setMaximumIterations (150);
  ndt2.setInputSource (filtered2_pc);
  ndt2.setInputTarget (filtered2_pc_old);
  ndt2.setOulierRatio(0.55);
  ndt2.align (*output_cloud, Tm);
  Tm = ndt2.getFinalTransformation ();
  std::cout << " score: " << ndt2.getFitnessScore (0.5) << std::endl;


  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt3;
  ndt3.setTransformationEpsilon (1e-6);
  ndt3.setStepSize (0.01);
  ndt3.setResolution (0.5);
  ndt3.setMaximumIterations (200);
  ndt3.setInputSource (filtered2_pc);
  ndt3.setInputTarget (filtered2_pc_old);
  ndt3.setOulierRatio(0.55);
  ndt3.align (*output_cloud, Tm);
  Tm = ndt3.getFinalTransformation ();
  std::cout << " score: " << ndt3.getFitnessScore (0.5) << std::endl;

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt4;
  ndt4.setTransformationEpsilon (1e-6);
  ndt4.setStepSize (0.01);
  ndt4.setResolution (0.25);
  ndt4.setMaximumIterations (200);
  ndt4.setInputSource (filtered2_pc);
  ndt4.setInputTarget (filtered2_pc_old);
  ndt4.setOulierRatio(0.55);
  ndt4.align (*output_cloud, Tm);
  Tm = ndt4.getFinalTransformation ();
  std::cout << " score: " << ndt4.getFitnessScore (0.5) << std::endl;
*/
  ///---------- End Matching ----------///


  tf::Vector3 origin;
  origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));
  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
        static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
        static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(tfqt);

  //cout << "x: " << Tm(0,3) << " y: " << Tm(1,3) << " z: " << Tm(2,3) << endl;
  tf::Matrix3x3 m(tfqt);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //cout << "yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << endl;

  curr_tf = curr_tf * transform;

  nav_msgs::Odometry odom;
  odom.header.stamp = pc_msg->header.stamp;
  odom.header.frame_id = "/origin"; // ro_origin
  odom.child_frame_id = "/lidar_odom";
  tf::poseTFToMsg(curr_tf, odom.pose.pose);

  odom.twist.twist.linear.x = Tm(0,3) / (pc_msg->header.stamp.toSec() - old_pc_time);
  odom.twist.twist.linear.y = Tm(1,3) / (pc_msg->header.stamp.toSec() - old_pc_time);
  odom.twist.twist.linear.z = sqrt(pow(odom.twist.twist.linear.x,2) + pow(odom.twist.twist.linear.y,2));

  lidar_odom_pub.publish(odom);

  //-- for debug --//
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = pc_msg->header.stamp; //radar_msg???
  transformStamped.header.frame_id = "ins_origin";
  transformStamped.child_frame_id = "lidar_odom";
  tf::transformTFToMsg(curr_tf, transformStamped.transform);
  br.sendTransform(transformStamped);
  //-- end debug --//


  //Viz(pc_old, pc, output_cloud); // filtered_

  ///---------- update old pc ----------///
  filtered_pc_old = filtered_pc;
  filtered2_pc_old = filtered2_pc;
	pc_old = pc;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_odometry");
  ros::NodeHandle n;

  curr_tf.setIdentity();

  //pc_pub = n.advertise<sensor_msgs::PointCloud2>("nuscenes_lidar_", 5);
  lidar_odom_pub = n.advertise<nav_msgs::Odometry> ("lidar_odom", 5);
  //error_pub = n.advertise<nav_msgs::Odometry> ("error", 5);

  ros::Subscriber sub = n.subscribe("nuscenes_lidar", 40000, Callback);

  /*
  int msg_flt_buffer = 40000;
  message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub(n, "/downsampled_semi_dense/radar_stack", msg_flt_buffer);
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(n, "/nuscenes_lidar", msg_flt_buffer);
  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(msg_flt_buffer), radar_sub, lidar_sub);
  no_cloud_sync_->registerCallback(boost::bind(&Callback, _1, _2));
  */

  ros::spin();
  return 0;
}
