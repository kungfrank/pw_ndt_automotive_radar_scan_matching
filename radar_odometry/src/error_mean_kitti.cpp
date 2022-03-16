#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <iostream>
using namespace std;

ros::Publisher pub;
float old_trans_error, old_theta_error;
float old_trans_error_sq, old_theta_error_sq;
int n=0;

void error_cb(nav_msgs::Odometry error){
	n++;
	float trans_error = error.pose.pose.position.z;
  float theta_error = abs(error.pose.pose.orientation.z); // in degree
	old_trans_error = old_trans_error + trans_error;
	old_theta_error = old_theta_error + theta_error;

  old_trans_error_sq = old_trans_error_sq + pow(trans_error,2);
  old_theta_error_sq = old_theta_error_sq + pow(theta_error/180*M_PI,2); // in radian

	//cout << ros::this_node::getName() << endl;

	cout << ros::this_node::getName() << " translation error: " << (old_trans_error/n)*100 << " %" << endl;
	cout << ros::this_node::getName() << " rotation error: " << (old_theta_error/n) << " deg/m" << endl;

  //cout << ros::this_node::getName() << " trans error var: " << old_trans_error_sq/n << endl;
  //cout << ros::this_node::getName() << " theta error var: " << old_theta_error_sq/n << endl;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "error_mean");
  ros::NodeHandle nh;

  ros::Subscriber sub_error = nh.subscribe ("/error", 5, error_cb);
	//pub = nh.advertise<std_msgs::Float64> ("/error mean", 1);

  ros::spin ();
}


