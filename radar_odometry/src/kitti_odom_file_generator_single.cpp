#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
using namespace std;

string folder_name = "/home/joinet";
string file_name = "gt.txt";

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

void Callback (const nav_msgs::OdometryConstPtr& odom)
{
  string file_path = folder_name+"/"+file_name;
  WriteToFile(odom, file_path);
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "kitti_odom_file_generator_single");
  ros::NodeHandle nh("~");


  if (nh.getParam("folder_name", folder_name)) ROS_INFO("Got 'folder_name' param: %s", folder_name.c_str());
  else ROS_WARN("Failed to get param 'folder_name', use default setting");

  if (nh.getParam("file_name", file_name)) ROS_INFO("Got 'file_name' param: %s", file_name.c_str());
  else ROS_WARN("Failed to get param 'file_name', use default setting");


  //system(("mkdir " + folder_name).c_str());

  ros::Subscriber sub = nh.subscribe("/radar_odom", 5000, Callback);
  ros::spin ();
}

