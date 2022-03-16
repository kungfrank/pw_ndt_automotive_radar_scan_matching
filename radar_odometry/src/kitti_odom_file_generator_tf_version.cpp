#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
using namespace std;

string folder_name = "/home/joinet";
string res_file_name = "res.txt";
string gt_file_name = "gt.txt";

//tf2_ros::TransformListener* tfListener_ptr;
tf2_ros::Buffer* tfBuffer_ptr;

bool init = false;
geometry_msgs::TransformStamped transformStamped_RO_origin_to_map;

void WriteOdomMsgToFile(const nav_msgs::OdometryConstPtr& odom_msg, string file_path){
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

void WriteTF2MsgToFile(geometry_msgs::Transform tf_msg, string file_path){
  double x = tf_msg.translation.x;
  double y = tf_msg.translation.y;
  double z = 0;
  tf::Quaternion q(
          tf_msg.rotation.x,
          tf_msg.rotation.y,
          tf_msg.rotation.z,
          tf_msg.rotation.w);
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

//---- 2020.08.03 This should be more accurate when calculating odometry error ----//
void WriteOdomMsgToFile_SE3(const nav_msgs::OdometryConstPtr& odom_msg, string file_path){
  double x = odom_msg->pose.pose.position.x;
  double y = odom_msg->pose.pose.position.y;
  double z = odom_msg->pose.pose.position.z;
  tf::Quaternion q(
          odom_msg->pose.pose.orientation.x,
          odom_msg->pose.pose.orientation.y,
          odom_msg->pose.pose.orientation.z,
          odom_msg->pose.pose.orientation.w);
  tf::Matrix3x3 rot_m(q);
  FILE *fp;
  fp = fopen(file_path.c_str(),"a");
  fprintf(fp,"%f %f %f %f %f %f %f %f %f %f %f %f\n",
          rot_m[0][0],rot_m[0][1],rot_m[0][2] ,x,
          rot_m[1][0],rot_m[1][1],rot_m[1][2] ,y,
          rot_m[2][0],rot_m[2][1],rot_m[2][2] ,z);
  fclose(fp);

}

void WriteTF2MsgToFile_SE3(geometry_msgs::Transform tf_msg, string file_path){
  double x = tf_msg.translation.x;
  double y = tf_msg.translation.y;
  double z = tf_msg.translation.z;
  tf::Quaternion q(
          tf_msg.rotation.x,
          tf_msg.rotation.y,
          tf_msg.rotation.z,
          tf_msg.rotation.w);
  tf::Matrix3x3 rot_m(q);

  FILE *fp;
  fp = fopen(file_path.c_str(),"a");
  fprintf(fp,"%f %f %f %f %f %f %f %f %f %f %f %f\n",
          rot_m[0][0],rot_m[0][1],rot_m[0][2] ,x,
          rot_m[1][0],rot_m[1][1],rot_m[1][2] ,y,
          rot_m[2][0],rot_m[2][1],rot_m[2][2] ,z);
  fclose(fp);

}


void Callback (const nav_msgs::OdometryConstPtr& odom)
{
  //-- write recived odom to file --//
  string file_path = folder_name + "/" + res_file_name;
  WriteOdomMsgToFile_SE3(odom, file_path);

  //-- find correspond gt and write to file --//
  if(!init){
    try{
      ////////////////////////////////// !!! Notice !!! origin VS ro_origin  //////////////////////////////////////
      transformStamped_RO_origin_to_map = tfBuffer_ptr->lookupTransform("origin", "map", odom->header.stamp, ros::Duration(0.0));
      std::cout << std::fixed; std::cout.precision(5);
 
      cout << "odom time: " << odom->header.stamp.toSec() << endl;
      cout << "GET ro_origin to map !!! \n";
      cout << "ro_origin to map time: " << transformStamped_RO_origin_to_map.header.stamp.toSec() << endl;
      init = true;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      //ros::Duration(0.1).sleep();
    }
  }
  geometry_msgs::TransformStamped transformStamped_map_to_car;

  try{
    transformStamped_map_to_car = tfBuffer_ptr->lookupTransform("map", "car", odom->header.stamp, ros::Duration(0.0));
    std::cout << std::fixed; std::cout.precision(5);
    /*
    cout << "======================= \n";
    cout << "odom time: " << odom->header.stamp.toSec() << endl;
    cout << "map to car time: " << transformStamped_map_to_car.header.stamp.toSec() << endl;
    */
    tf::Transform tf_RO_origin_to_map, tf_map_to_car;
    tf::transformMsgToTF(transformStamped_RO_origin_to_map.transform, tf_RO_origin_to_map);
    tf::transformMsgToTF(transformStamped_map_to_car.transform, tf_map_to_car);

    geometry_msgs::Transform transform_RO_origin_to_car;
    tf::transformTFToMsg(tf_RO_origin_to_map * tf_map_to_car, transform_RO_origin_to_car);

    WriteTF2MsgToFile_SE3(transform_RO_origin_to_car, folder_name + "/" + gt_file_name);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    //ros::Duration(0.1).sleep();
  }

  ///-- We cannot use this directly, sometimes this will fail and skip one groundturth --///
  /*
  try{
    transformStamped = tfBuffer_ptr->lookupTransform("ro_origin", "car", odom->header.stamp, ros::Duration(0.0));
    std::cout << std::fixed; std::cout.precision(5);
    cout << "======================= \n";
    cout << "odom time: " << odom->header.stamp.toSec() << endl;
    cout << "ro_origin to car time: " << transformStamped.header.stamp.toSec() << endl;
    WriteTF2MsgToFile(transformStamped, folder_name + "/" + gt_file_name);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    //ros::Duration(0.1).sleep();
  }
  */

}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "kitti_odom_file_generator_single_tf_version");
  ros::NodeHandle nh("~");

  tf2_ros::Buffer tfBuffer(ros::Duration(100)); // save all tf in 100 sec
  tf2_ros::TransformListener tfListener(tfBuffer);
  tfBuffer_ptr = &tfBuffer;

  if (nh.getParam("folder_name", folder_name)) ROS_INFO("Got 'folder_name' param: %s", folder_name.c_str());
  else ROS_WARN("Failed to get param 'folder_name', use default setting");

  if (nh.getParam("res_file_name", res_file_name)) ROS_INFO("Got 'res_file_name' param: %s", res_file_name.c_str());
  else ROS_WARN("Failed to get param 'res_file_name', use default setting");

  if (nh.getParam("gt_file_name", gt_file_name)) ROS_INFO("Got 'gt_file_name' param: %s", gt_file_name.c_str());
  else ROS_WARN("Failed to get param 'gt_file_name', use default setting");

  ros::Subscriber sub = nh.subscribe("/radar_odom", 5000, Callback);
  ros::spin ();
}
