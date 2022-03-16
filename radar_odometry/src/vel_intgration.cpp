/********************************************************
This is the node integrate velocity into odometry.
-------------------------------
INPUT Topic:(nav_msgs::Odometry)
	/gt_vel
-------------------------------
OUTPUT Topic:(nav_msgs::Odometry)
	/gt_ego_motion

by Frank Kung 2019 Dec
*********************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <math.h>
using namespace std;

ros::Publisher pub;
tf::TransformListener* tran;

ros::Time old_t;
tf::Transform origin_to_center_old;
bool init_flag = true;
bool init = false;

tf::Transform getTransform(float x,float y, float theta){
	tf::Transform temp_tf;
	tf::Quaternion q; q.setRPY(0, 0, theta);
  temp_tf.setRotation(q);
	temp_tf.setOrigin( tf::Vector3(x, y, 0.0) );
  return temp_tf;
}

void vel_cb(nav_msgs::Odometry vel){

if(init_flag == true){old_t = vel.header.stamp;	init_flag = false;}
else{

	ros::Time curr_t = vel.header.stamp;
	double delta_t = (curr_t - old_t).toSec();

	double Vx = vel.twist.twist.linear.x;
	double Vy =  vel.twist.twist.linear.y;
	double Omega = vel.twist.twist.angular.z;

	double delta_x = Vx * delta_t;
	double delta_y = Vy * delta_t;
	double delta_theta = Omega/180*M_PI * delta_t;
  /*
  Eigen::MatrixXd cov_mat;
  cov_mat.resize(3, 3);
  cov_mat(0,0) = vel.twist.covariance[0]; cov_mat(0,1) = vel.twist.covariance[1]; cov_mat(0,2) = vel.twist.covariance[5];
  cov_mat(1,0) = vel.twist.covariance[6]; cov_mat(1,1) = vel.twist.covariance[7]; cov_mat(1,2) = vel.twist.covariance[11];
  cov_mat(2,0) = vel.twist.covariance[30]; cov_mat(2,1) = vel.twist.covariance[31]; cov_mat(2,2) = vel.twist.covariance[35];
  */
  double std_x = sqrt(vel.twist.covariance[0] * delta_t * delta_t);
  double std_y = sqrt(vel.twist.covariance[7] * delta_t * delta_t);
  double std_theta = sqrt(vel.twist.covariance[35] * delta_t * delta_t);

  //cout << "std_x: " << std_x << endl;
  //cout << "std_y: " << std_y << endl;
  //cout << "std_trans: " << sqrt(std_x*std_x + std_y*std_y) << endl;
  //cout << "std_theta: " << std_theta << endl;

/*
	cout << "delta_t: " << delta_t << endl;
	cout << "delta_x: " << delta_x << endl;
	cout << "delta_y: " << delta_y << endl;
	cout << "delta_theta: " << delta_theta << endl;
*/

	tf::Transform delta_tf;
	tf::Quaternion q; q.setRPY(0, 0, delta_theta);
  delta_tf.setRotation(q);
	delta_tf.setOrigin( tf::Vector3(delta_x, delta_y, 0.0) );

	tf::Transform origin_to_center;
	origin_to_center = origin_to_center_old * delta_tf;

  tf::Transform vel_local;
  q.setRPY(0, 0, Omega/180*M_PI);
  vel_local.setRotation(q);
  vel_local.setOrigin( tf::Vector3(Vx, Vy, 0.0) );

  tf::Transform origin_to_center_rot;
  origin_to_center_rot = origin_to_center;  origin_to_center_rot.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Transform vel_from_origin;
  vel_from_origin = origin_to_center_rot * vel_local;

  //tf::Transform vel_cov_from_origin;
  //vel_cov_from_origin

  q = vel_from_origin.getRotation();
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

	nav_msgs::Odometry odom;
	odom.header.stamp = vel.header.stamp;
	odom.header.frame_id = "/origin";
  odom.child_frame_id = "/car";
  tf::poseTFToMsg(origin_to_center, odom.pose.pose);  // pose w.r.t origin coordinate

  odom.pose.covariance[0] = std_x*std_x;
  odom.pose.covariance[7] = std_y*std_y;
  odom.pose.covariance[14] = 0;
  odom.pose.covariance[35] = std_theta*std_theta*1;

  odom.twist = vel.twist; // local velocity
  pub.publish(odom);

	// Update Time
	old_t = curr_t;
	origin_to_center_old = origin_to_center;
}
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "velocity_integration");
  ros::NodeHandle nh;

	cout << "Start velocity_integration " << endl;

  ros::Subscriber sub = nh.subscribe ("/gt_vel", 2, vel_cb);
  pub = nh.advertise<nav_msgs::Odometry> ("/gt_ego_motion", 1);

	origin_to_center_old.setIdentity();

  ros::spin ();
}


