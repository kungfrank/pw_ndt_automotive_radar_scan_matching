/********************************************************

[WARNING] This code fail in TF listening part

*********************************************************/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <string>
#include <math.h>
using namespace std;

ros::Publisher odom_error_pub;
ros::Publisher odom_error_short_pub;
tf::TransformListener* tran;
//
int frame_count = 0;
nav_msgs::Odometry gt_odom_kf_old;
nav_msgs::Odometry odom_kf_old;
//
nav_msgs::Odometry gt_odom_old;
float travel_dis = 0;
//
int frame_count_num = 1;
//
double GetYawFromQ(geometry_msgs::Quaternion rot){
  tf::Quaternion q(
      rot.x,
      rot.y,
      rot.z,
      rot.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
	return yaw;
}

void Callback (nav_msgs::Odometry::Ptr odom)
{

  tf2_ros::Buffer tfBuffer(ros::Duration(2));
  tf2_ros::TransformListener tfListener(tfBuffer);

	std::cout << std::fixed; std::cout.precision(5);
  cout << "odom time: " << odom->header.stamp.toSec() << endl;

	tf::StampedTransform gt_transform;
	try{
    tran->waitForTransform("/origin", "/car", ros::Time(odom->header.stamp.toSec()), ros::Duration(0.5));
		tran->lookupTransform("/origin", "/car", ros::Time(odom->header.stamp.toSec()), gt_transform); //odom->header.stamp
	}
	catch (tf::TransformException& ex) {	ROS_WARN("%s",ex.what());}

/*
	geometry_msgs::TransformStamped gt_transform;
  try{
    gt_transform = tfBuffer.lookupTransform("origin", "car",odom->header.stamp);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
*/

	nav_msgs::Odometry::Ptr gt_odom;
	//gt_odom->header.stamp = gt_transform.stamp_;
  //tf::poseTFToMsg(gt_transform, gt_odom->pose.pose);

  cout << "gt_odom time: " << gt_transform.stamp_.toSec() << endl;
/*
	float x_error, y_error, trans_error;

	x_error = gt_odom->pose.pose.position.x - odom->pose.pose.position.x;
	y_error = gt_odom->pose.pose.position.y - odom->pose.pose.position.y;
	trans_error = sqrt(pow(x_error,2)+pow(y_error,2));

	nav_msgs::Odometry odom_error;
	odom_error.header.stamp = gt_odom->header.stamp;
	odom_error.header.frame_id = " ";
	odom_error.child_frame_id = " ";
	odom_error.pose.pose.position.x = x_error;
	odom_error.pose.pose.position.y = y_error;
	odom_error.pose.pose.position.z = trans_error; 

	double yaw_gt = GetYawFromQ(gt_odom->pose.pose.orientation);
  double yaw = GetYawFromQ(odom->pose.pose.orientation);
	odom_error.pose.pose.orientation.z = (yaw - yaw_gt)/M_PI*180;

	odom_error_pub.publish(odom_error);
	cout << "trans error: " << trans_error << endl;

	float delta_x = gt_odom->pose.pose.position.x - gt_odom_old.pose.pose.position.x;
	float delta_y = gt_odom->pose.pose.position.y - gt_odom_old.pose.pose.position.y;
	travel_dis = sqrt(pow(delta_x,2)+pow(delta_y,2)) + travel_dis;

	//cout << "travel_dis: " << travel_dis << endl;

	frame_count++;
	if(frame_count == frame_count_num){

		double delta_x = odom->pose.pose.position.x - odom_kf_old.pose.pose.position.x;
		double delta_y = odom->pose.pose.position.y - odom_kf_old.pose.pose.position.y;
		double delta_yaw = GetYawFromQ(odom->pose.pose.orientation) - GetYawFromQ(odom_kf_old.pose.pose.orientation);
		delta_yaw = delta_yaw/M_PI*180;

		double delta_x_gt = gt_odom->pose.pose.position.x - gt_odom_kf_old.pose.pose.position.x;
		double delta_y_gt = gt_odom->pose.pose.position.y - gt_odom_kf_old.pose.pose.position.y;
		double delta_yaw_gt = GetYawFromQ(gt_odom->pose.pose.orientation) - GetYawFromQ(gt_odom_kf_old.pose.pose.orientation);
		delta_yaw_gt = delta_yaw_gt/M_PI*180;

		nav_msgs::Odometry odom_error_short;
		odom_error_short.header.stamp = gt_odom->header.stamp;
		odom_error_short.header.frame_id = " ";
		odom_error_short.child_frame_id = " ";
		odom_error_short.pose.pose.position.x = abs(delta_x - delta_x_gt);
		odom_error_short.pose.pose.position.y = abs(delta_y - delta_y_gt);
		odom_error_short.pose.pose.position.z = abs(sqrt(pow(delta_x,2)+pow(delta_y,2))-sqrt(pow(delta_x_gt,2)+pow(delta_y_gt,2))); 
		odom_error_short.pose.pose.orientation.z = delta_yaw - delta_yaw_gt;
		odom_error_short_pub.publish(odom_error_short);

		frame_count = 0;
		gt_odom_kf_old = *gt_odom;
		odom_kf_old = *odom;
	}

	gt_odom_old = *gt_odom;
*/

}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "odom_error_calculation_tf_version");
  ros::NodeHandle nh("~");

  if (nh.getParam("frame_count_num", frame_count_num)) ROS_INFO("Got 'frame_count_num' param: %d", frame_count_num);
  else ROS_WARN("Failed to get param 'frame_count_num', use default setting");

	ros::Subscriber sub = nh.subscribe("/radar_ego_motion", 1000, Callback);

  tf::TransformListener lr(ros::Duration(100));
  tran=&lr;

  odom_error_pub = nh.advertise<nav_msgs::Odometry> ("accumulate_odom_error", 1);
  odom_error_short_pub = nh.advertise<nav_msgs::Odometry> ("interval_odom_error", 1);

  ros::spin ();
}

