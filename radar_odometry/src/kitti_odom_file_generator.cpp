#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
using namespace std;

int msg_flt_buffer = 2000;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> NoCloudSyncPolicy;

string folder_name = "/home/joinet/radar_ws/src/radar_odometry/src/result";
string gt_file_name = "gt.txt";
string res_file_name = "res.txt";


void WriteToFile(const nav_msgs::OdometryConstPtr& odom_msg, string file_path){
  double x = odom_msg->pose.pose.position.x;
  double y = odom_msg->pose.pose.position.y;
  double z = 0;
  tf::Quaternion q(
          odom_msg->pose.pose.orientation.x,
          odom_msg->pose.pose.orientation.y,
          odom_msg->pose.pose.orientation.z,
          odom_msg->pose.pose.orientation.w);
  tf::Matrix3x3 rot_m(q);
/*
  std::cout << std::fixed; std::cout.precision(3);
  cout << rot_m[0][0] << " " << rot_m[0][1] << " " << rot_m[0][2] << " " << x << endl;
  cout << rot_m[1][0] << " " << rot_m[1][1] << " " << rot_m[1][2] << " " << y << endl;
  cout << rot_m[2][0] << " " << rot_m[2][1] << " " << rot_m[2][2] << " " << z << endl;
  cout << " -------------------------------------- " << endl;
*/
  FILE *fp;
  fp = fopen(file_path.c_str(),"a");
  fprintf(fp,"%f %f %f %f %f %f %f %f %f %f %f %f\n",
          rot_m[0][0],rot_m[0][1],0.0 ,x,
          rot_m[1][0],rot_m[1][1],0.0 ,y,
          0.0        ,0.0        ,1.0 ,z);
  fclose(fp);

}

void Callback (const nav_msgs::OdometryConstPtr& gt_odom, const nav_msgs::OdometryConstPtr& odom)
{

  string gt_file_path = folder_name+"/"+gt_file_name;

  WriteToFile(gt_odom, gt_file_path);

  string res_file_path = folder_name+"/"+res_file_name;

  WriteToFile(odom, res_file_path);
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "kitti_odom_file_generator");
  ros::NodeHandle nh("~");

  if (nh.getParam("msg_flt_buffer", msg_flt_buffer)) ROS_INFO("Got 'msg_flt_buffer' param: %d", msg_flt_buffer);
  else ROS_WARN("Failed to get param 'msg_flt_buffer', use default setting");

  if (nh.getParam("folder_name", folder_name)) ROS_INFO("Got 'folder_name' param: %s", folder_name);
  else ROS_WARN("Failed to get param 'folder_name', use default setting");

  if (nh.getParam("gt_file_name", gt_file_name)) ROS_INFO("Got 'gt_file_name' param: %s", gt_file_name);
  else ROS_WARN("Failed to get param 'gt_file_name', use default setting");

  if (nh.getParam("res_file_name", res_file_name)) ROS_INFO("Got 'res_file_name' param: %s", res_file_name);
  else ROS_WARN("Failed to get param 'res_file_name', use default setting");

  //system(("mkdir " + folder_name).c_str());

  message_filters::Subscriber<nav_msgs::Odometry> sub_gt(nh, "/gt_odom", 1000);
  message_filters::Subscriber<nav_msgs::Odometry> sub_result(nh, "/radar_ego_motion", 1000);

  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(msg_flt_buffer), sub_gt, sub_result);
  no_cloud_sync_->registerCallback(boost::bind(&Callback, _1, _2));

  ros::spin ();
}

