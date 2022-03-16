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

tf::Transform getTransform(float x,float y, float theta){
	tf::Transform temp_tf;
	tf::Quaternion q; q.setRPY(0, 0, theta);
  temp_tf.setRotation(q);
	temp_tf.setOrigin( tf::Vector3(x, y, 0.0) );
  return temp_tf;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gt_vel_publisher");
  ros::NodeHandle node;

  ros::Publisher gt_vel_pub = 
    node.advertise<nav_msgs::Odometry>("/gt_vel", 10);

  ros::Publisher gt_odom_pub = 
    node.advertise<nav_msgs::Odometry>("/gt_odom", 10);

  tf::TransformListener listener;

	ros::Time old_t;
	tf::Transform transform_gt_center_old;
	origin_to_center_old.setIdentity();
	transform_gt_center_old.setIdentity();

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
		//-- caculate gt speed --//
		double gt_car_vel_x; // vx
		double gt_car_vel_y; // vy
		double gt_car_omega; // angular speed

		tf::Transform tf_f_by_f; // frame by frame transformation
		try{
			listener.waitForTransform("/map", "/car",
					                      ros::Time::now(), ros::Duration(0.01));
			listener.lookupTransform("/map", "/car", ros::Time(0), transform_gt_center);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s -- by gt_vel_pub node",ex.what());
			ros::Duration(1.0).sleep();
		}

		ros::Time curr_t = transform_gt_center.stamp_;
		double delta_t = (curr_t - old_t).toSec();

		cout << "curr_t: " << curr_t << endl;
		cout << "delta_t: " << delta_t << endl;

		tf_f_by_f = transform_gt_center_old.inverse()*transform_gt_center;

		double delta_x = (double)tf_f_by_f.getOrigin().x();
		double delta_y = (double)tf_f_by_f.getOrigin().y();
		tf::Quaternion q = tf_f_by_f.getRotation();
	  tf::Matrix3x3 m(q);
	  double roll, pitch, yaw;
	  m.getRPY(roll, pitch, yaw);
		double delta_theta = yaw;

		gt_car_vel_x = delta_x/delta_t; // vx
		gt_car_vel_y = delta_y/delta_t; // vy
		gt_car_omega = delta_theta/delta_t; // omega

		cout << "delta_x: " << delta_x << endl;
		cout << "delta_y: " << delta_y << endl;
		cout << endl;

		//-- gt_vel Msg --//
	  nav_msgs::Odometry vel_odom;
		vel_odom.header.frame_id = "/car";
		vel_odom.header.stamp = transform_gt_center.stamp_;
		vel_odom.twist.twist.linear.x = gt_car_vel_x;
		vel_odom.twist.twist.linear.y = gt_car_vel_y;
		vel_odom.twist.twist.angular.z = gt_car_omega/M_PI*180;

		//-- gt_odom Msg --//
		origin_to_center = origin_to_center_old * tf_f_by_f;
	  nav_msgs::Odometry odom;
		odom.header.stamp = transform_gt_center.stamp_;
		odom.header.frame_id = "/origin";
		odom.child_frame_id = "/origin";
		tf::poseTFToMsg(origin_to_center, odom.pose.pose);
/*
		cout << " origin_to_center_old: " << origin_to_center_old.getOrigin().x()<<" "<<origin_to_center_old.getOrigin().y() << endl;
		cout << " tf_f_by_f: " << tf_f_by_f.getOrigin().x()<<" "<<tf_f_by_f.getOrigin().y() << endl;
		cout << " origin_to_center: " << origin_to_center.getOrigin().x()<<" "<<origin_to_center.getOrigin().y() << endl;
		cout << " odom.pose.pose: " << odom.pose.pose.position.x<<" "<<odom.pose.pose.position.y << endl;
*/
		if(delta_t > 0.0){ // delta_t > 0.001
	  	gt_vel_pub.publish(vel_odom);
			// gt_odom_pub.publish(odom);
/*
			cout << "delta_t: " << delta_t << endl;
			cout << "gt_car_vel_x: " << gt_car_vel_x << endl;
			cout << "gt_car_vel_y: " << gt_car_vel_y << endl;
			cout << "gt_car_omega: " << gt_car_omega/M_PI*180 << endl;
			cout << endl;
*/
		}
		origin_to_center_old = origin_to_center;
		transform_gt_center_old = transform_gt_center;
		old_t = curr_t;

	  rate.sleep();
		}
  }
  return 0;
};

