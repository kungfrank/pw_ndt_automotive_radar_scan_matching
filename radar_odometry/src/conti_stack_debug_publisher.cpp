#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <conti_radar/Measurement.h>
#include <conti_radar/ContiRadar.h>

#include <sstream>

ros::Publisher f_inlier_pub;
ros::Publisher fr_inlier_pub;
ros::Publisher fl_inlier_pub;
ros::Publisher br_inlier_pub;
ros::Publisher bl_inlier_pub;

ros::Publisher pub;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "conti_stack_debug_publisher");
  ros::NodeHandle nh;

  f_inlier_pub  = nh.advertise<conti_radar::Measurement> ("radar_front_inlier", 1);
  fr_inlier_pub = nh.advertise<conti_radar::Measurement> ("radar_front_right_inlier", 1);
  fl_inlier_pub = nh.advertise<conti_radar::Measurement> ("radar_front_left_inlier", 1);
  br_inlier_pub = nh.advertise<conti_radar::Measurement> ("radar_back_right_inlier", 1);
  bl_inlier_pub = nh.advertise<conti_radar::Measurement> ("radar_back_left_inlier", 1);

  pub = nh.advertise<nav_msgs::Odometry> ("vel", 1);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std::cout << "loop!! \n";
    conti_radar::Measurement radar_measurement;
    conti_radar::ContiRadar radar_point;
    for(int i=0; i<10; i++){
      radar_point.longitude_dist = 0; // x
      radar_point.lateral_dist = i * 5; // y
      radar_measurement.points.push_back(radar_point);
    }
    radar_measurement.header.stamp = ros::Time::now();
    radar_measurement.header.frame_id = "/nuscenes_radar_front";
    std::cout << "pub front radar \n";
    f_inlier_pub.publish(radar_measurement);

    conti_radar::Measurement radar_measurement_empty;
    radar_measurement_empty.header.stamp = ros::Time::now();
    radar_measurement_empty.header.frame_id = "/nuscenes_radar_front_right";
    fr_inlier_pub.publish(radar_measurement_empty);
    radar_measurement_empty.header.frame_id = "/nuscenes_radar_front_left";
    fl_inlier_pub.publish(radar_measurement_empty);
    radar_measurement_empty.header.frame_id = "/nuscenes_radar_back_right";
    br_inlier_pub.publish(radar_measurement_empty);
    radar_measurement_empty.header.frame_id = "/nuscenes_radar_back_left";
    bl_inlier_pub.publish(radar_measurement_empty);

    nav_msgs::Odometry odom;
    odom.header.frame_id = "/car";
    odom.header.stamp = ros::Time::now();
    odom.twist.twist.linear.x = 4.0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0; // deg

    /// assign covariance value ///
    odom.twist.covariance[0] = 0.2;  odom.twist.covariance[1] = 0;  odom.twist.covariance[5] = 0;
    odom.twist.covariance[6] = 0; 	odom.twist.covariance[7] = 0.2;  odom.twist.covariance[11] = 0;
    odom.twist.covariance[30] = 0; odom.twist.covariance[31] = 0;  odom.twist.covariance[35] = (0.0/180*M_PI) * (0.0/180*M_PI);
    odom.twist.covariance[14] = 99999;
    odom.twist.covariance[21] = 99999;
    odom.twist.covariance[28] = 99999;

    std::cout << "pub vel \n";
    pub.publish(odom);


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
