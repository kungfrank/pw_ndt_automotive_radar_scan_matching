#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <iostream>
using namespace std;

tf::Transform origin_to_center;
tf::Transform origin_to_center_old;
tf::StampedTransform transform_gt_center;
bool init = false;

int main(int argc, char** argv){
  ros::init(argc, argv, "gt_odom_publisher");
  ros::NodeHandle node;

  ros::Publisher gt_odom_pub = 
    node.advertise<nav_msgs::Odometry>("/gt_odom", 20);

  tf::TransformListener listener;

	ros::Time old_t;
	tf::Transform transform_gt_center_old;
	origin_to_center_old.setIdentity();
	transform_gt_center_old.setIdentity();

	double travel_dis = 0;
	double delta_dis = 0;

  ros::Rate rate(200.0);
  while (node.ok()){
		if(!init){
		  try{
				listener.waitForTransform("/map", "/car",
				                          ros::Time::now(), ros::Duration(0.1));
		    listener.lookupTransform("/map", "/car",  
		                             ros::Time(0), transform_gt_center);

				old_t = transform_gt_center.stamp_;
				transform_gt_center_old = transform_gt_center;
				init = true;
		  }
		  catch (tf::TransformException ex){
		    ROS_WARN("%s",ex.what());
		    ros::Duration(1.0).sleep();
		  }
		}
		else{

		tf::Transform tf_f_by_f; // frame by frame transformation
		try{
			listener.waitForTransform("/map", "/car",
					                      ros::Time::now(), ros::Duration(0.01));
			listener.lookupTransform("/map", "/car", ros::Time(0), transform_gt_center);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s -- by gt_odom_pub node",ex.what());
			ros::Duration(1.0).sleep();
		}

		//ros::Time curr_t = transform_gt_center.stamp_;
		//double delta_t = (curr_t - old_t).toSec();

		tf_f_by_f = transform_gt_center_old.inverse()*transform_gt_center;

		//cout << "tf_f_by_f: " << tf_f_by_f.getOrigin().x() << ", " << tf_f_by_f.getOrigin().y() << endl;
		delta_dis = sqrt(pow(tf_f_by_f.getOrigin().x(),2)+pow(tf_f_by_f.getOrigin().y(),2));
		travel_dis = travel_dis + delta_dis;
		//cout << "Tavel Distance: " << travel_dis << " m" << endl;

		//-- gt_odom Msg --//
		origin_to_center = origin_to_center_old * tf_f_by_f;
	  nav_msgs::Odometry odom;
		odom.header.stamp = transform_gt_center.stamp_;
		odom.header.frame_id = "/origin";
		odom.child_frame_id = "/origin";
		tf::poseTFToMsg(origin_to_center, odom.pose.pose);
		gt_odom_pub.publish(odom);

		origin_to_center_old = origin_to_center;
		transform_gt_center_old = transform_gt_center;
		//old_t = curr_t;

	  rate.sleep();
		}
  }
  return 0;
};

