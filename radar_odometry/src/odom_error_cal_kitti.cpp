#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
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
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> NoCloudSyncPolicy;
//
int frame_count = 0;
nav_msgs::Odometry gt_odom_kf_old;
nav_msgs::Odometry odom_kf_old;
//
nav_msgs::Odometry gt_odom_old;
double travel_dis = 0;
double travel_dis_old = 0;
//
int msg_flt_buffer = 7;
int frame_count_num = 1;
//
bool init = false;
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

void Callback (const nav_msgs::OdometryConstPtr& gt_odom, const nav_msgs::OdometryConstPtr& odom)
{
	std::cout << std::fixed; std::cout.precision(5);

	//-- The time difference here can cost error caculation unprecise --//

  //cout << "gt_odom time: " << gt_odom->header.stamp.toSec() << endl;
  //cout << "odom time: " << odom->header.stamp.toSec() << endl;

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

	float delta_x = gt_odom->pose.pose.position.x - gt_odom_old.pose.pose.position.x;
	float delta_y = gt_odom->pose.pose.position.y - gt_odom_old.pose.pose.position.y;
	travel_dis = sqrt(pow(delta_x,2)+pow(delta_y,2)) + travel_dis;

	cout << "travel_dis: " << travel_dis << endl;

	if(!init){
		gt_odom_kf_old = *gt_odom;
		odom_kf_old = *odom;
		init = true;
	}

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

		double delta_gt_travel_dis = sqrt(pow(delta_x_gt,2)+pow(delta_y_gt,2));

    if(travel_dis - travel_dis_old > 1){
			nav_msgs::Odometry odom_error_short;
			odom_error_short.header.stamp = gt_odom->header.stamp;
			odom_error_short.header.frame_id = " ";
			odom_error_short.child_frame_id = " ";
			odom_error_short.pose.pose.position.x = abs(delta_x - delta_x_gt) / delta_x_gt;
			odom_error_short.pose.pose.position.y = abs(delta_y - delta_y_gt) / delta_y_gt;
			odom_error_short.pose.pose.position.z = abs(sqrt(pow(delta_x,2)+pow(delta_y,2))-delta_gt_travel_dis) / delta_gt_travel_dis; 
			odom_error_short.pose.pose.orientation.z = (delta_yaw - delta_yaw_gt) / delta_gt_travel_dis;

			odom_error_short_pub.publish(odom_error_short);

			gt_odom_kf_old = *gt_odom;
			odom_kf_old = *odom;
			travel_dis_old = travel_dis;
		}

		frame_count = 0;
	}
	gt_odom_old = *gt_odom;
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "odom_error_calculation");
  ros::NodeHandle nh("~");

  if (nh.getParam("msg_flt_buffer", msg_flt_buffer)) ROS_INFO("Got 'msg_flt_buffer' param: %d", msg_flt_buffer);
  else ROS_WARN("Failed to get param 'msg_flt_buffer', use default setting");
  if (nh.getParam("frame_count_num", frame_count_num)) ROS_INFO("Got 'frame_count_num' param: %d", frame_count_num);
  else ROS_WARN("Failed to get param 'frame_count_num', use default setting");

  message_filters::Subscriber<nav_msgs::Odometry> sub_gt(nh, "/gt_odom", 1000);
  message_filters::Subscriber<nav_msgs::Odometry> sub_result(nh, "/radar_ego_motion", 1000);

  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(msg_flt_buffer), sub_gt, sub_result);
  no_cloud_sync_->registerCallback(boost::bind(&Callback, _1, _2));

  odom_error_pub = nh.advertise<nav_msgs::Odometry> ("accumulate_odom_error", 10);
  odom_error_short_pub = nh.advertise<nav_msgs::Odometry> ("interval_odom_error", 10);

  ros::spin ();
}

