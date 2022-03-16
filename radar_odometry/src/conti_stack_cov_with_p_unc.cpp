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

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ro_msg/Cov2D.h>
#include <ro_msg/Cov2DArrayStamped.h>
#include <ro_msg/IntArrayStamped.h>

using namespace std;

//-- ROS global variable --//
ros::Publisher valid_pub;
ros::Publisher invalid_pub;
ros::Publisher stack_pub;

ros::Publisher cov_pub;
ros::Publisher index_array_pub;



//-- global variable --//
double min_t;
vector<nav_msgs::Odometry::ConstPtr> vec_odom_msg;
vector<sensor_msgs::PointCloud2::Ptr> vec_val_pc2_msg;
vector<sensor_msgs::PointCloud2::Ptr> vec_inval_pc2_msg;
vector<sensor_msgs::PointCloud2::Ptr> vec_pc2_msg;

int init_frame_num = 0;
int stack_frame_num = 5;

//tf::StampedTransform transform_map_to_origin;
geometry_msgs::TransformStamped transform_map_to_RO_origin_msg;
tf2_ros::Buffer* tfBuffer_ptr;
tf2_ros::TransformBroadcaster* broadcaster_ptr;

typedef message_filters::sync_policies::ApproximateTime<conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        conti_radar::Measurement,
																												conti_radar::Measurement,
																												nav_msgs::Odometry> NoCloudSyncPolicy;

struct PointCloudWithCov {
    sensor_msgs::PointCloud2::Ptr pc_msg_ptr;
    vector<Eigen::Matrix2d> vec_cov;
};

ro_msg::IntArrayStamped GetPCIndexNum(vector<sensor_msgs::PointCloud2::Ptr> vec_pc2_msg){
  ro_msg::IntArrayStamped index_array_msg;
  int num=0;
  for(int i=0; i<vec_pc2_msg.size(); i++){
    num = num +  vec_pc2_msg[i]->width;
    index_array_msg.data.push_back(num);
  }
  return index_array_msg;
}

sensor_msgs::PointCloud2::Ptr RecursiveStackingFrame(const sensor_msgs::PointCloud2::Ptr& pc_old, const sensor_msgs::PointCloud2::Ptr& pc_new, Eigen::Matrix4f tf_);

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

Eigen::Matrix4f getTdTransform(float td, const nav_msgs::Odometry& vel){
  float vx,vy,omega,delta_x,delta_y,delta_theta;
  vx = vel.twist.twist.linear.x;
  vy = vel.twist.twist.linear.y;
  omega = (vel.twist.twist.angular.z)/180*M_PI;
  delta_x = vx * td;
  delta_y = vy * td;
  delta_theta = omega * td;
  Eigen::Matrix4f transform = getTransform(delta_x,delta_y,delta_theta);
  return transform;
}

Eigen::Matrix3d getVariance(float td, const nav_msgs::Odometry& vel){
  Eigen::Matrix3d cov_mat;

  cov_mat(0,0) = vel.twist.covariance[0];  cov_mat(0,1) = vel.twist.covariance[1];  cov_mat(0,2) = vel.twist.covariance[5];
  cov_mat(1,0) = vel.twist.covariance[6];  cov_mat(1,1) = vel.twist.covariance[7];  cov_mat(1,2) = vel.twist.covariance[11];
  cov_mat(2,0) = vel.twist.covariance[30];  cov_mat(2,1) = vel.twist.covariance[31];  cov_mat(2,2) = vel.twist.covariance[35];

  //cout << "cov_mat: \n" << cov_mat <<endl;

  /////// Use larger covariance matrix .....
  //cov_mat = cov_mat * 100;

  return cov_mat * static_cast<double> (td*td);
}

Eigen::Matrix3d getFixedVariance(float td, const nav_msgs::Odometry& vel){
  Eigen::Matrix3d cov_mat;
  ///------------------- fix uncertainty !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -------------------///
  //cov_mat(0,0) = 0.0002;  cov_mat(0,1) = 0;  cov_mat(0,2) = 0;
  //cov_mat(1,0) = 0;  cov_mat(1,1) = 0.0002;  cov_mat(1,2) = 0;
  //cov_mat(2,0) = 0;  cov_mat(2,1) = 0;  cov_mat(2,2) = 6.4e-6;

  cov_mat(0,0) = 0.000;  cov_mat(0,1) = 0;  cov_mat(0,2) = 0;
  cov_mat(1,0) = 0;  cov_mat(1,1) = 0.000;  cov_mat(1,2) = 0;
  cov_mat(2,0) = 0;  cov_mat(2,1) = 0;  cov_mat(2,2) = 0.0001;// sigma = 0.01 -> 0.57degree

  cov_mat(2,2) = 0.000;

  return cov_mat;
}

double getTdRotation(float td, const nav_msgs::Odometry& vel){
  float omega, delta_theta;
  omega = (vel.twist.twist.angular.z)/180*M_PI;
  delta_theta = omega * td;
  return delta_theta;
}

Eigen::Matrix2d getCovFromXY(double x, double y){
  Eigen::Matrix2d point_covar = Eigen::Matrix2d::Zero ();
  Eigen::Matrix2d jacobian = Eigen::Matrix2d::Zero ();
  Eigen::Matrix2d var = Eigen::Matrix2d::Zero ();
  double sigma_r = 0.1/3; // 3sigma -> 99% // accuracy is 0.1 ~ 0.4
  double sigma_theta = 0.5/3; //0.1/3;                // accuracy is 0.1 ~ 1 degree
  var(0,0) = pow(sigma_r,2);  var(1,1) = pow(sigma_theta/180*M_PI,2);

  ////---- Point's Uncertainty ----////
  double r = std::sqrt(pow(x,2) + pow(y,2));
  double phi = atan2(y, x);
  jacobian(0,0) = cos(phi); jacobian(0,1) = -r*sin(phi);
  jacobian(1,0) = sin(phi); jacobian(1,1) = r*cos(phi);
  point_covar = jacobian * var * jacobian.transpose();
  ////---- End ----////

  return point_covar;
}

vector<Eigen::Matrix2d> getInitCovFromPcMsg(const sensor_msgs::PointCloud2::Ptr& pc_msg){
  vector<Eigen::Matrix2d> vec_init_cov;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pc_msg, *pc);
  float x, y;
  for(int i=0; i<pc->size(); i++){
    vec_init_cov.push_back(getCovFromXY(pc->points[i].x, pc->points[i].y));
  }
  //cout << "vec_init_cov: " << vec_init_cov.size() << endl;
  return vec_init_cov;
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

vector<Eigen::Matrix2d> PropagateCov(sensor_msgs::PointCloud2::Ptr pc_msg,
                                     vector<Eigen::Matrix2d> vec_pc_cov,
                                     const sensor_msgs::PointCloud2::Ptr& pc_new,
                                     double phi_,
                                     Eigen::Matrix3d var_mat_ego){
  vector<Eigen::Matrix2d> new_vec_pc_cov;
  new_vec_pc_cov.resize(vec_pc_cov.size());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pc_msg, *pc);

  int x, y;
  // pc->size() should = vec_pc_cov.size()
  //if(pc->size() != vec_pc_cov.size()){cout << "SIZE ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;}
  //cout << "pc->size(): " << pc->size() << endl;
  for(int i=0; i < pc->size(); i++){
    x = pc->points[i].x;
    y = pc->points[i].y;

    Eigen::MatrixXd jacobian(2,3);
    jacobian(0,0) = 1;  jacobian(0,1) = 0;  jacobian(0,2) = -x * sin(phi_) - y * cos(phi_);
    jacobian(1,0) = 0;  jacobian(1,1) = 1;  jacobian(1,2) = x * cos(phi_) - y * sin(phi_);

    Eigen::Matrix2d var_mat_point = jacobian * var_mat_ego * jacobian.transpose();

    //cout << "phi_: " << phi_ << endl;
    //cout << "var_mat_point: \n" << var_mat_point << endl;

    Eigen::Matrix2d Gt_mat;
    Gt_mat(0,0) = cos(phi_);  Gt_mat(0,1) = -sin(phi_);
    Gt_mat(1,0) = sin(phi_);  Gt_mat(1,1) = cos(phi_);

    new_vec_pc_cov[i] = Gt_mat*vec_pc_cov[i]*Gt_mat.transpose() + var_mat_point; // Update Variance of Point
  }

  vector<Eigen::Matrix2d> vec_init_cov = getInitCovFromPcMsg(pc_new);

  vector<Eigen::Matrix2d> new_vec_pc_cov_;
  new_vec_pc_cov_.reserve( new_vec_pc_cov.size() + vec_init_cov.size() ); // preallocate memory

  ///////////// !!! This is origin version !!! ////////////////

  //new_vec_pc_cov_.insert( new_vec_pc_cov_.end(), new_vec_pc_cov.begin(), new_vec_pc_cov.end() );
  //new_vec_pc_cov_.insert( new_vec_pc_cov_.end(), vec_init_cov.begin(), vec_init_cov.end() );

  new_vec_pc_cov_.insert( new_vec_pc_cov_.end(), vec_init_cov.begin(), vec_init_cov.end() );
  new_vec_pc_cov_.insert( new_vec_pc_cov_.end(), new_vec_pc_cov.begin(), new_vec_pc_cov.end() );

  //for(int i=0; i<pc_new->width; i++){ new_vec_pc_cov.push_back(Eigen::Matrix2d::Zero());} // need to include init unc
  return new_vec_pc_cov_;
}

struct PointCloudWithCov RecursiveStackFrame(vector<nav_msgs::Odometry::ConstPtr> vec_odom_msg,
                                              vector<sensor_msgs::PointCloud2::Ptr> vec_pc2_msg,
                                              sensor_msgs::PointCloud2::Ptr pc,
                                              vector<Eigen::Matrix2d> vec_pc_cov,
                                              double old_t,
                                              nav_msgs::Odometry old_v){

  sensor_msgs::PointCloud2::Ptr new_pc(new sensor_msgs::PointCloud2);
  vector<Eigen::Matrix2d> new_vec_pc_cov;

  if(vec_odom_msg.size() == stack_frame_num){
    new_pc = vec_pc2_msg[0];
    old_t = vec_pc2_msg[0]->header.stamp.toSec();
    old_v = *vec_odom_msg[0];
    new_vec_pc_cov = getInitCovFromPcMsg(vec_pc2_msg[0]);
    // for(int i=0; i<vec_pc2_msg[0]->width; i++){ new_vec_pc_cov.push_back(Eigen::Matrix2d::Zero());} // need to include init unc
  }
  else{
    double curr_t = vec_pc2_msg[0]->header.stamp.toSec();
    Eigen::Matrix4f tf_ = getTdTransform(old_t-curr_t, old_v);
    new_pc = RecursiveStackingFrame(pc, vec_pc2_msg[0], tf_);

    //Eigen::Matrix3d var_mat_ego = getFixedVariance(curr_t-old_t, old_v);
    Eigen::Matrix3d var_mat_ego = getVariance(curr_t-old_t, old_v);
    //cout << "var_ego: \n" << var_ego <<endl;
    double delta_theta = getTdRotation(old_t-curr_t, old_v);
    new_vec_pc_cov = PropagateCov(pc, vec_pc_cov, vec_pc2_msg[0], delta_theta, var_mat_ego);
  }
  old_t = vec_pc2_msg[0]->header.stamp.toSec();
  old_v = *vec_odom_msg[0];
  vec_odom_msg.erase (vec_odom_msg.begin());
  vec_pc2_msg.erase (vec_pc2_msg.begin());

  if(vec_odom_msg.size() != 0){
    return RecursiveStackFrame(vec_odom_msg, vec_pc2_msg, new_pc, new_vec_pc_cov, old_t, old_v);
  }
  else{
    struct PointCloudWithCov pc_with_cov;
    pc_with_cov.pc_msg_ptr = new_pc;
    pc_with_cov.vec_cov = new_vec_pc_cov;
    return pc_with_cov;
  }
}

sensor_msgs::PointCloud2::Ptr RecursiveStackingFrame(const sensor_msgs::PointCloud2::Ptr& pc_old, const sensor_msgs::PointCloud2::Ptr& pc_new, Eigen::Matrix4f tf_){
  /// stacking pc_old to pc_new frame with tf_ transform ///

  sensor_msgs::PointCloud2::Ptr temp(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr pc_out(new sensor_msgs::PointCloud2);
  pcl_ros::transformPointCloud(tf_, *pc_old, *temp);
  pcl::concatenatePointCloud(*pc_new, *temp, *pc_out);
  return pc_out;
}


sensor_msgs::PointCloud2::Ptr stackFrame(vector<nav_msgs::Odometry::ConstPtr> vec_odom_msg,
                                         vector<sensor_msgs::PointCloud2::Ptr> vec_pc2_msg){
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

    pcl_ros::transformPointCloud(transform_old*transform_td, *(vec_pc2_msg[i]), temp);
    pcl::concatenatePointCloud(output, temp, output);

    transform_old = transform_old * transform_td;
    old_t = curr_t;
  }
  transform_old = Eigen::Matrix4f::Identity();

  const sensor_msgs::PointCloud2::Ptr& output_ptr = boost::make_shared<sensor_msgs::PointCloud2>(output);
  return output_ptr;
}


void MergeCallback (const conti_radar::MeasurementConstPtr& f_input,
                    const conti_radar::MeasurementConstPtr& fl_input,
                    const conti_radar::MeasurementConstPtr& fr_input,
                    const conti_radar::MeasurementConstPtr& bl_input,
                    const conti_radar::MeasurementConstPtr& br_input,
                    const nav_msgs::OdometryConstPtr& vel) 
{
  //------------ Debug info ------------//

  cout << "---------------------------------------- \n";
  std::cout << std::fixed; std::cout.precision(5);
  cout << "f time: " << f_input->header.stamp.toSec() << endl;
  cout << "fl time: " << fl_input->header.stamp.toSec() << endl;
  cout << "fr time: " << fr_input->header.stamp.toSec() << endl;
  cout << "bl time: " << bl_input->header.stamp.toSec() << endl;
  cout << "br time: " << br_input->header.stamp.toSec() << endl;
  cout << "vel time: " << vel->header.stamp.toSec() << endl;

  //------------ Debug info end ------------//

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
  /*
  cout << "f size:" << f_input->points.size() << " ";
  cout << "fr size:" << fr_input->points.size() << " "; cout << "fl size:" << fl_input->points.size() << " ";
  cout << "br size:" << br_input->points.size() << " "; cout << "bl size:" << bl_input->points.size() << endl;
  */
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

  sensor_msgs::PointCloud2::Ptr pc_all(new sensor_msgs::PointCloud2);
  pcl::concatenatePointCloud(*valid_pc, *invalid_pc, *pc_all);

	//-- Publish valid_pc and invalid_pc --//
  valid_pub.publish(*valid_pc);
  invalid_pub.publish(*invalid_pc);

  //cout << "[merge] pc_all size h: " << pc_all->height << endl;
  //cout << "[merge] pc_all size w: " << pc_all->width << endl;

  //static tf::TransformBroadcaster tb;

	//-- Stack radar scan --//
	init_frame_num++;
	if(init_frame_num <= stack_frame_num){
		vec_odom_msg.push_back(vel);
    vec_pc2_msg.push_back(pc_all);
	}
	else{
    ///-- Publish Radar Odometry Origin (TF1 version, use ros::Time(0) is not good way to do this) --///
    /*
    if(init_frame_num == stack_frame_num+1){
      try{
        //cout << "get ro_origin !!!" << endl;
        tf::TransformListener listener;
        listener.waitForTransform("/map", "/car",
                                  ros::Time(0), ros::Duration(0.05));
        listener.lookupTransform("/map", "/car",
                                 ros::Time(0), transform_map_to_origin);
        //cout << transform_map_to_origin.getOrigin().x() << endl;
      }
      catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::Duration(0.5).sleep();
      }
    }
    tb.sendTransform(tf::StampedTransform(transform_map_to_origin, ros::Time(min_t), "/map", "/ro_origin"));
    */
    ///-- End Publishing Radar Odometry Origin --///

    ///-- Publish Radar Odometry Origin (TF2) --///

    if(init_frame_num == stack_frame_num+1){
      //cout << "get ro_origin !!!" << endl;
      try{
        transform_map_to_RO_origin_msg = tfBuffer_ptr->lookupTransform("map", "car", ros::Time(min_t), ros::Duration(0.0));
        transform_map_to_RO_origin_msg.header.frame_id = "map";
        transform_map_to_RO_origin_msg.child_frame_id = "ro_origin";
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.05).sleep();
      }
    }
    //cout << "broadcaster map_to_ro_origin  !!!" << endl;
    transform_map_to_RO_origin_msg.header.stamp = ros::Time(min_t);
    //cout << transform_map_to_RO_origin_msg.header.stamp.toSec() << endl;
    broadcaster_ptr->sendTransform(transform_map_to_RO_origin_msg);

    ///-- End Publishing Radar Odometry Origin --///

		vec_odom_msg.erase (vec_odom_msg.begin());
    vec_pc2_msg.erase (vec_pc2_msg.begin());
		vec_odom_msg.push_back(vel);
    vec_pc2_msg.push_back(pc_all);

    sensor_msgs::PointCloud2::Ptr pc(new sensor_msgs::PointCloud2);
    vector<Eigen::Matrix2d> vec_pc_cov;
    double old_t;
    nav_msgs::Odometry old_v;

    struct PointCloudWithCov stack_pc_with_cov;

    stack_pc_with_cov = RecursiveStackFrame(vec_odom_msg, vec_pc2_msg, pc, vec_pc_cov, old_t, old_v);

    //sensor_msgs::PointCloud2::Ptr stack_pc(new sensor_msgs::PointCloud2);

    stack_pub.publish(stack_pc_with_cov.pc_msg_ptr);
    //cout << "[merge] stack_pc size h: " << stack_pc_with_cov.pc_msg_ptr->height << endl;
    //cout << "[merge] stack_pc size w: " << stack_pc_with_cov.pc_msg_ptr->width << endl;

    ro_msg::Cov2DArrayStamped cov_msg;
    cov_msg.header.stamp = ros::Time(min_t);
    ro_msg::Cov2D temp;
    for(int i=0; i<stack_pc_with_cov.vec_cov.size(); i++){
      temp.covariance[0] = stack_pc_with_cov.vec_cov[i](0,0);
      temp.covariance[1] = stack_pc_with_cov.vec_cov[i](0,1);
      temp.covariance[2] = stack_pc_with_cov.vec_cov[i](1,0);
      temp.covariance[3] = stack_pc_with_cov.vec_cov[i](1,1);
      cov_msg.data.push_back(temp);
    }

    cov_pub.publish(cov_msg);


    /// This is for old ndt2d scan matching ///
    ro_msg::IntArrayStamped index_array_msg;
    index_array_msg = GetPCIndexNum(vec_pc2_msg);
    index_array_msg.header.stamp = ros::Time(min_t);
    index_array_pub.publish(index_array_msg);
    ///

	}
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "conti_stack_cov");
  ROS_INFO("Start conti_stack_cov");
  ros::NodeHandle nh("~");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster broadcaster;
  tfBuffer_ptr = &tfBuffer;
  broadcaster_ptr = &broadcaster;

  if (nh.getParam("stack_frame_num", stack_frame_num)) ROS_INFO("Got 'stack_frame_num' param: %d", stack_frame_num);
  else ROS_WARN("Failed to get param 'stack_frame_num', use default setting");

  message_filters::Subscriber<conti_radar::Measurement> sub_f(nh, "/radar_front_inlier", 1000);
  message_filters::Subscriber<conti_radar::Measurement> sub_fl(nh, "/radar_front_left_inlier", 1000);
  message_filters::Subscriber<conti_radar::Measurement> sub_fr(nh, "/radar_front_right_inlier", 1000);
  message_filters::Subscriber<conti_radar::Measurement> sub_bl(nh, "/radar_back_left_inlier", 1000);
  message_filters::Subscriber<conti_radar::Measurement> sub_br(nh, "/radar_back_right_inlier", 1000);
  message_filters::Subscriber<nav_msgs::Odometry> sub_vel(nh, "/vel", 1000);

  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(1000), sub_f, sub_fl, sub_fr, sub_bl, sub_br, sub_vel);
  no_cloud_sync_->registerCallback(boost::bind(&MergeCallback, _1, _2, _3, _4, _5, _6));

  valid_pub = nh.advertise<sensor_msgs::PointCloud2> ("radar_inlier_valid", 1);
  invalid_pub = nh.advertise<sensor_msgs::PointCloud2> ("radar_inlier_invalid", 1);

	//-- large pub buffer is set for python msg flt	--//
	
  stack_pub = nh.advertise<sensor_msgs::PointCloud2> ("radar_stack", 100000); //100
  cov_pub = nh.advertise<ro_msg::Cov2DArrayStamped> ("radar_stack_cov", 100000);

  index_array_pub = nh.advertise<ro_msg::IntArrayStamped> ("radar_stack_index", 100000);


  ros::spin ();
}

