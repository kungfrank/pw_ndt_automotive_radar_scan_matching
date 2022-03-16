/********************************************************
This is the node for NDT.
-------------------------------
INPUT Topic:
  PointCloud: (sensor_msgs::PointCloud2)
		/input_pc
  Velocity integration: (nav_msgs::Odometry)
		/radar_ego_motion
-------------------------------
OUTPUT Topic:
	Odomtery: (nav_msgs::Odometry)
		ndt_odom

by Frank Kung 2019 Dec
*********************************************************/

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <ro_msg/IntArrayStamped.h>
#include <ro_msg/Cov2D.h>
#include <ro_msg/Cov2DArrayStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <string>

using namespace std;

ros::Publisher pub_stack_pc, pub_stack_index, pub_cov;

int msg_flt_buffer = 100;
int count_ = 0;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                        ro_msg::IntArrayStamped,
                                                        ro_msg::Cov2DArrayStamped> NoCloudSyncPolicy;



void Callback(const sensor_msgs::PointCloud2ConstPtr& input,
              const ro_msg::IntArrayStampedConstPtr& stack_index_msg,
              const ro_msg::Cov2DArrayStampedConstPtr& pc_cov_msg){
  count_ = count_+1;
  if(count_ == 3){
    pub_stack_pc.publish(*input);
    pub_stack_index.publish(*stack_index_msg);
    pub_cov.publish(*pc_cov_msg);
    count_ = 0;
  }

}

int main (int argc, char** argv)
{

  ros::init (argc, argv, "4hz_semi_dense");
  ros::NodeHandle nh("~");

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_stack_pc(nh, "/input_pc", 1000);
  message_filters::Subscriber<ro_msg::IntArrayStamped> sub_stack_index(nh, "/radar_stack_index", 1000);
  message_filters::Subscriber<ro_msg::Cov2DArrayStamped> sub_cov(nh, "/radar_stack_cov", 1000);
  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(msg_flt_buffer), sub_stack_pc, sub_stack_index, sub_cov);
  no_cloud_sync_->registerCallback(boost::bind(&Callback, _1, _2, _3));

  pub_stack_pc = nh.advertise<sensor_msgs::PointCloud2> ("input_pc", 10);
  pub_stack_index = nh.advertise<ro_msg::IntArrayStamped> ("radar_stack_index", 10);
  pub_cov = nh.advertise<ro_msg::Cov2DArrayStamped> ("radar_stack_cov", 10);

  ros::spin ();
}



