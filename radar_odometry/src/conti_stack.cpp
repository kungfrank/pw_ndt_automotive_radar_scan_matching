/********************************************************
This is the node for stacking multiple radar scan and compensate td between 5 radar scan using velocity from radar ego-motion.
-------------------------------
INPUT Topic:
  Radar Scan: (conti_radar::Measurement)
		/radar_front_inlier
		/radar_front_left_inlier
		/radar_front_right_inlier
		/radar_back_left_inlier
		/radar_back_right_inlier
  Velocity: (nav_msgs::Odometry)
		/vel
-------------------------------
OUTPUT Topic:
  Singal Radar Pointcloud: (sensor_msgs::PointCloud2)
		radar_inlier_valid
		radar_inlier_invalid
  Stacked Radar Pointcloud: (sensor_msgs::PointCloud2)
		radar_stack
		radar_stack_radius_filter (X)

by Frank Kung 2019 Dec
*********************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <conti_radar/Measurement.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <string>
#include <math.h>

#include <ro_msg/IntArrayStamped.h>

using namespace std;

//-- ROS global variable --//
ros::Publisher valid_pub;
ros::Publisher invalid_pub;
ros::Publisher stack_pub;
ros::Publisher stack_flt_pub;

ros::Publisher num_pub;
ros::Publisher index_array_pub;

//-- global variable --//
double min_t;
vector<nav_msgs::Odometry::ConstPtr> vec_odom_msg;
vector<sensor_msgs::PointCloud2::Ptr> vec_val_pc2_msg;
vector<sensor_msgs::PointCloud2::Ptr> vec_inval_pc2_msg;

int init_frame_num = 0;
int stack_frame_num = 10;

typedef message_filters::sync_policies::ApproximateTime<conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        conti_radar::Measurement,
																												conti_radar::Measurement,
																												nav_msgs::Odometry> NoCloudSyncPolicy;

Eigen::Matrix4f getTransform(float x,float y, float theta){
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform (0,0) = std::cos (theta);
  transform (0,1) = -sin(theta);
  transform (1,0) = sin (theta);
  transform (1,1) = std::cos (theta);
  transform (0,3) = x;
  transform (1,3) = y;
  return transform;
}

Eigen::Matrix4f getTdTransform(float td, const nav_msgs::OdometryConstPtr& vel){

	float vx,vy,omega,delta_x,delta_y,delta_theta;

	vx = vel->twist.twist.linear.x;
	vy = vel->twist.twist.linear.y;
	omega = (vel->twist.twist.angular.z)/180*M_PI;

	delta_x = vx * td;
	delta_y = vy * td;
	delta_theta = omega * td;

  Eigen::Matrix4f transform = getTransform(delta_x,delta_y,delta_theta);
	return transform;
}

void conti_filter(const conti_radar::MeasurementConstPtr& conti_input, sensor_msgs::PointCloud2::Ptr ros_pc2, sensor_msgs::PointCloud2::Ptr ros_pc2_i){

	pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>); // valid pc
	pcl::PCLPointCloud2 pc2;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_i(new pcl::PointCloud<pcl::PointXYZI>); // invalid pc
	pcl::PCLPointCloud2 pc2_i;

	pcl::PointXYZI point;
	//cout << conti_input->points.size() << endl;
	for(int i=0; i< (conti_input->points.size()); i++){
		if ((conti_input->points[i].invalid_state) == 0x00){
			point.x = conti_input->points[i].longitude_dist;
			point.y = conti_input->points[i].lateral_dist;
			point.z = 0;
			point.intensity = conti_input->points[i].rcs;
			pc->points.push_back(point);
		}
		else{
			point.x = conti_input->points[i].longitude_dist;
			point.y = conti_input->points[i].lateral_dist;
			point.z = 0;
			point.intensity = conti_input->points[i].rcs;
			pc_i->points.push_back(point);
		}
	}
	//cout << "valid num: " << pc->points.size() << endl;
	pcl::toPCLPointCloud2(*pc, pc2);
  pcl_conversions::fromPCL(pc2, *ros_pc2);
	ros_pc2->is_dense = false;
	ros_pc2->header = conti_input->header; ///////////////////

	pcl::toPCLPointCloud2(*pc_i, pc2_i);
  pcl_conversions::fromPCL(pc2_i, *ros_pc2_i);
	ros_pc2_i->is_dense = false;
	ros_pc2_i->header = conti_input->header; ///////////////////
}

//////////////////////-- Time Difference Compensate Part --//////////////////////
sensor_msgs::PointCloud2::Ptr merge( const sensor_msgs::PointCloud2::Ptr& f,
 																const sensor_msgs::PointCloud2::Ptr& fl,
 																const sensor_msgs::PointCloud2::Ptr& fr,
 																const sensor_msgs::PointCloud2::Ptr& bl,
 																const sensor_msgs::PointCloud2::Ptr& br,
 																const nav_msgs::OdometryConstPtr& vel){
	//-- get min time --//
	double f_t = f->header.stamp.toSec();
	double fl_t = fl->header.stamp.toSec();
	double fr_t = fr->header.stamp.toSec();
	double bl_t = bl->header.stamp.toSec();
	double br_t = br->header.stamp.toSec();
	double list[] = {f_t,fl_t,fr_t,bl_t,br_t};

	min_t = *std::min_element(list,list+5);
/*
	std::cout << std::fixed; std::cout.precision(5);
  cout << "min time: " << min_t << endl;
	cout << "vel->header.stamp.toSec(): " << vel->header.stamp.toSec() << endl;
*/
	//-- marge to local map frame --//
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 temp;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_td = Eigen::Matrix4f::Identity();

  transform = getTransform(3.41199994087, 0, 0.003);
	transform_td = getTdTransform(f_t-min_t, vel);
  pcl_ros::transformPointCloud(transform_td*transform, *f, temp);
  pcl::concatenatePointCloud(output, temp, output);

  transform = getTransform(2.42199993134, 0.800000011921, 1.542);
	transform_td = getTdTransform(fl_t-min_t, vel);
  pcl_ros::transformPointCloud(transform_td*transform, *fl, temp);
  pcl::concatenatePointCloud(output, temp, output);

  transform = getTransform(2.42199993134, -0.800000011921, -1.588);
	transform_td = getTdTransform(fr_t-min_t, vel);
  pcl_ros::transformPointCloud(transform_td*transform, *fr, temp);
  pcl::concatenatePointCloud(output, temp, output);

  transform = getTransform(-0.561999976635, 0.628000020981, 3.044);
	transform_td = getTdTransform(bl_t-min_t, vel);
  pcl_ros::transformPointCloud(transform_td*transform, *bl, temp);
  pcl::concatenatePointCloud(output, temp, output);

  transform = getTransform(-0.561999976635, -0.617999970913, -3.074);
	transform_td = getTdTransform(br_t-min_t, vel);
  pcl_ros::transformPointCloud(transform_td*transform, *br, temp);
  pcl::concatenatePointCloud(output, temp, output);

	const sensor_msgs::PointCloud2::Ptr& output_ptr = boost::make_shared<sensor_msgs::PointCloud2>(output);

	return output_ptr;
}
//////////////////////-- Time Difference Compensate Part End --//////////////////////

sensor_msgs::PointCloud2::Ptr stackFrame(vector<nav_msgs::Odometry::ConstPtr> vec_odom_msg,
																				 vector<sensor_msgs::PointCloud2::Ptr> vec_val_pc2_msg,
																				 vector<sensor_msgs::PointCloud2::Ptr> vec_inval_pc2_msg){
/*
	cout << "stackFrame" << endl;
	cout << "vec_odom_msg.size(): " << vec_odom_msg.size() << endl;
	cout << "vec_val_pc2_msg.size(): " << vec_val_pc2_msg.size() << endl;
	cout << "vec_inval_pc2_msg.size(): " << vec_inval_pc2_msg.size() << endl;
*/

  sensor_msgs::PointCloud2 output;

	double old_t = 0;
	bool stack_init = false;
	Eigen::Matrix4f transform_old = Eigen::Matrix4f::Identity();
	//cout << "stackframe! " << endl;
	for(int i=vec_odom_msg.size()-1; i>=0; i--){
		if(stack_init == false){old_t = vec_odom_msg[i]->header.stamp.toSec();	stack_init = true;}
		double curr_t = vec_odom_msg[i]->header.stamp.toSec();

		sensor_msgs::PointCloud2 temp;
		Eigen::Matrix4f transform_td = Eigen::Matrix4f::Identity();

		transform_td = getTdTransform(curr_t-old_t, vec_odom_msg[i]);
		pcl_ros::transformPointCloud(transform_old*transform_td, *(vec_val_pc2_msg[i]), temp);
		pcl::concatenatePointCloud(output, temp, output);
		pcl_ros::transformPointCloud(transform_old*transform_td, *(vec_inval_pc2_msg[i]), temp);
		pcl::concatenatePointCloud(output, temp, output);

		transform_old = transform_old * transform_td;


		old_t = curr_t;
	}
	transform_old = Eigen::Matrix4f::Identity();

	const sensor_msgs::PointCloud2::Ptr& output_ptr = boost::make_shared<sensor_msgs::PointCloud2>(output);
	return output_ptr;
}

sensor_msgs::PointCloud2::Ptr radiusOutlierRemoval(const sensor_msgs::PointCloud2::Ptr& pc_in_ptr){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2 pc_out;
	
	pcl::fromROSMsg(*pc_in_ptr, *cloud);
  // build the filter
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(1);
  outrem.setMinNeighborsInRadius(3);
  // apply filter
  outrem.filter (*cloud_filtered);
	pcl::toROSMsg(*cloud_filtered, pc_out);

	const sensor_msgs::PointCloud2::Ptr& pc_out_ptr = boost::make_shared<sensor_msgs::PointCloud2>(pc_out);
	return pc_out_ptr;
}

int GetPointNumAfterNFrame(int N,
                            vector<sensor_msgs::PointCloud2::Ptr> vec_val_pc2_msg,
                            vector<sensor_msgs::PointCloud2::Ptr> vec_inval_pc2_msg){
  int num = 0;
  for(int i=vec_val_pc2_msg.size()-1; i>=N; i--){
    //cout << "index[" << i << "] :" << vec_val_pc2_msg[i]->width + vec_inval_pc2_msg[i]->width << "\n";
    num = num + vec_val_pc2_msg[i]->width + vec_inval_pc2_msg[i]->width;
  }
  return num;
}

ro_msg::IntArrayStamped GetPCIndexNum(vector<sensor_msgs::PointCloud2::Ptr> vec_val_pc2_msg,
                                      vector<sensor_msgs::PointCloud2::Ptr> vec_inval_pc2_msg){
  ro_msg::IntArrayStamped index_array_msg;
  int num=0;
  for(int i=vec_val_pc2_msg.size()-1; i>=0; i--){
    //cout << "index[" << i << "] :" << vec_val_pc2_msg[i]->width + vec_inval_pc2_msg[i]->width << "\n";
    num = num + vec_val_pc2_msg[i]->width + vec_inval_pc2_msg[i]->width;
    index_array_msg.data.push_back(num);
  }
  return index_array_msg;
}


void MergeCallback (const conti_radar::MeasurementConstPtr& f_input,
                    const conti_radar::MeasurementConstPtr& fl_input,
                    const conti_radar::MeasurementConstPtr& fr_input,
                    const conti_radar::MeasurementConstPtr& bl_input,
                    const conti_radar::MeasurementConstPtr& br_input,
                    const nav_msgs::OdometryConstPtr& vel) 
{
	// valid pointcloud
  sensor_msgs::PointCloud2::Ptr f(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fl(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fr(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr bl(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr br(new sensor_msgs::PointCloud2);
	// invalid pointcloud
  sensor_msgs::PointCloud2::Ptr f_i(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fl_i(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fr_i(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr bl_i(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr br_i(new sensor_msgs::PointCloud2);

	//-- seperate valid and invalid pointcloud from 5 radar scan --//
	conti_filter(f_input, f, f_i);
	conti_filter(fl_input, fl, fl_i);
	conti_filter(fr_input, fr, fr_i);
	conti_filter(bl_input, bl, bl_i);
	conti_filter(br_input, br, br_i);

	//-- td compensate valid and invalid pointcloud and merge them into 1 radar scan --//
  sensor_msgs::PointCloud2::Ptr valid_pc;
	valid_pc = merge(f, fl, fr, bl, br, vel);
  valid_pc->header.stamp = ros::Time(min_t);
  valid_pc->header.frame_id = "/car";

  sensor_msgs::PointCloud2::Ptr invalid_pc;
	invalid_pc = merge(f_i, fl_i, fr_i, bl_i, br_i, vel);
  invalid_pc->header.stamp = ros::Time(min_t);
  invalid_pc->header.frame_id = "/car";

	//-- Publish valid_pc and invalid_pc --//
  valid_pub.publish(*valid_pc);
  invalid_pub.publish(*invalid_pc);

	//-- Stack radar scan --//
	init_frame_num++;
	if(init_frame_num <= stack_frame_num){
		//cout << "111" << endl;
		vec_odom_msg.push_back(vel);
		vec_val_pc2_msg.push_back(valid_pc);
		vec_inval_pc2_msg.push_back(invalid_pc);

	}
	else{

		vec_odom_msg.erase (vec_odom_msg.begin());
		vec_val_pc2_msg.erase (vec_val_pc2_msg.begin());
		vec_inval_pc2_msg.erase (vec_inval_pc2_msg.begin());
		vec_odom_msg.push_back(vel);
		vec_val_pc2_msg.push_back(valid_pc);
		vec_inval_pc2_msg.push_back(invalid_pc);

		sensor_msgs::PointCloud2::Ptr stack_pc(new sensor_msgs::PointCloud2);
		stack_pc = stackFrame(vec_odom_msg, vec_val_pc2_msg, vec_inval_pc2_msg);

		stack_pub.publish(stack_pc);

		stack_pc = radiusOutlierRemoval(stack_pc);
		stack_flt_pub.publish(stack_pc);

    /// This is for img scan matching ///
		int num=0;
		num = GetPointNumAfterNFrame(10,vec_val_pc2_msg, vec_inval_pc2_msg);
		geometry_msgs::PointStamped num_msg;
		num_msg.header = stack_pc->header;
		num_msg.point.x = num;
		num_pub.publish(num_msg);

    /// This is for ndt2d scan matching ///
    ro_msg::IntArrayStamped index_array_msg;
    index_array_msg = GetPCIndexNum(vec_val_pc2_msg, vec_inval_pc2_msg);
    index_array_msg.header = stack_pc->header;
    index_array_pub.publish(index_array_msg);
    ///

		sensor_msgs::PointCloud2::Ptr new_pc(new sensor_msgs::PointCloud2);
		stack_pc = new_pc;
	}
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "conti_stack");
	ROS_INFO("Start conti_stack");
  ros::NodeHandle nh("~");

  if (nh.getParam("stack_frame_num", stack_frame_num)) ROS_INFO("Got 'stack_frame_num' param: %d", stack_frame_num);
  else ROS_WARN("Failed to get param 'stack_frame_num', use default setting");

  message_filters::Subscriber<conti_radar::Measurement> sub_f(nh, "/radar_front_inlier", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_fl(nh, "/radar_front_left_inlier", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_fr(nh, "/radar_front_right_inlier", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_bl(nh, "/radar_back_left_inlier", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_br(nh, "/radar_back_right_inlier", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_vel(nh, "/vel", 1);

  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(6), sub_f, sub_fl, sub_fr, sub_bl, sub_br, sub_vel);
  no_cloud_sync_->registerCallback(boost::bind(&MergeCallback, _1, _2, _3, _4, _5, _6));

  valid_pub = nh.advertise<sensor_msgs::PointCloud2> ("radar_inlier_valid", 1);
  invalid_pub = nh.advertise<sensor_msgs::PointCloud2> ("radar_inlier_invalid", 1);

  stack_pub = nh.advertise<sensor_msgs::PointCloud2> ("radar_stack", 1);
  stack_flt_pub = nh.advertise<sensor_msgs::PointCloud2> ("radar_stack_radius_filter", 1);

	///
  num_pub = nh.advertise<geometry_msgs::PointStamped> ("/radar_point_num_after_10", 1);
  index_array_pub = nh.advertise<ro_msg::IntArrayStamped> ("/radar_stack_index", 1);
	///

  ros::spin ();
}

