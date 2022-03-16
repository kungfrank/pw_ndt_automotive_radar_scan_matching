/********************************************************
This is the implementation of the probabilistic radar ego-motion on Nuscenes dataset following paper "Probabilistic ego-motion estimation using multiple automotive radar sensors", in 2017 RAS
-------------------------------
INPUT Topic:
  Radar Scan: (conti_radar::Measurement)
    /radar_front
    /radar_front_left
    /radar_front_right
    /radar_back_left
    /radar_back_right
  Velocity: (nav_msgs::Odometry)
    /vel
-------------------------------
OUTPUT Topic:
  Singal Radar Pointcloud: (sensor_msgs::PointCloud2)
    radar_combined
  Radar Pointcloud weighted by doppler estimation: (ro_msg::IntArrayStamped)
    radar_weight


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
ros::Publisher radar_pub;
ros::Publisher radar_weight_pub;

//-- global variable --//
double min_t;

float sigma_v = 0.4;// param in paper = 0.4
// 0.014;//(m/s)
float sigma_theta = 0.5/180*M_PI; // param in paper
//0.3/180 * M_PI;

typedef message_filters::sync_policies::ApproximateTime<conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        //nav_msgs::Odometry,
                                                        //nav_msgs::Odometry,
                                                        //nav_msgs::Odometry,
                                                        //nav_msgs::Odometry,
                                                        //nav_msgs::Odometry,
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

Eigen::MatrixXd calcVjxVjy(Eigen::MatrixXd ego_mat, float r_x, float r_y){
  Eigen::MatrixXd Vj;
  Eigen::MatrixXd S_mat;
  S_mat.resize(2, 3); // 2 x 3
  S_mat(0,0) = -r_y; S_mat(0,1) = 1.0; S_mat(0,2) = 0.0;
  S_mat(1,0) = r_x;  S_mat(1,1) = 0.0; S_mat(1,2) = 1.0;
  Vj = S_mat * ego_mat;
  return Vj;
}

float calc_weight_by_doppler_estimation(conti_radar::ContiRadar radar_point,
                                        Eigen::MatrixXd radar_sensor_vel,
                                        float alpha){
  float x = radar_point.longitude_dist;
  float y = radar_point.lateral_dist;
  //float vD = sqrt(pow(radar_point.lateral_vel,2) + pow(radar_point.longitude_vel,2));
  float theta = atan2(y,x);

  Eigen::Vector2d radial_vector(x, y);
  Eigen::Vector2d radial_vector_normalized = radial_vector.normalized();
  Eigen::Vector2d vel(radar_point.longitude_vel, radar_point.lateral_vel);
  float vD = vel.dot(radial_vector_normalized); //projection

  //cout << "v_theta: " << atan2(radar_point.lateral_vel, radar_point.longitude_vel) << " theta: " << theta << endl;


  float vx_s = radar_sensor_vel(0,0);
  float vy_s = radar_sensor_vel(1,0);

  //-- Expected Doppler velocity --//
  float vD_ = -(vx_s * cos(theta + alpha) + vy_s * sin(theta + alpha)); //-------------------(16)
  float error = pow(vD_ - vD, 2);

  //-- Corresponding variance --//

  float Jacobian = vx_s*(-sin(theta)*cos(alpha) - cos(theta)*sin(alpha))
                  + vy_s*(cos(theta)*cos(alpha) - sin(theta)*sin(alpha));
                  //-------------------(16)'s derivative by theta

  float var_v = pow(sigma_v,2);
  float var_theta = pow(sigma_theta,2);
  float var = var_v + var_theta * pow(Jacobian,2); //-------------------(17)

  float weight = exp(-error/(2*var)) * (1/sqrt(2*M_PI*var)); //-------------------(18)

  return weight;
}

void conti_converter(const conti_radar::MeasurementConstPtr& conti_input,
                     sensor_msgs::PointCloud2::Ptr ros_pc2,
                     Eigen::MatrixXd radar_sensor_vel,
                     float alpha){

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PCLPointCloud2 pc2;

  pcl::PointXYZI point;
  //cout << conti_input->points.size() << endl;
  for(int i=0; i< (conti_input->points.size()); i++){
    point.x = conti_input->points[i].longitude_dist;
    point.y = conti_input->points[i].lateral_dist;
    point.z = 0;
    //--- doppler_weight ---//
    point.intensity = calc_weight_by_doppler_estimation(conti_input->points[i], radar_sensor_vel, alpha);
    //point.intensity = 1;

    //point.intensity = conti_input->points[i].rcs;
    pc->points.push_back(point);
  }
  pcl::toPCLPointCloud2(*pc, pc2);
  pcl_conversions::fromPCL(pc2, *ros_pc2);
  ros_pc2->is_dense = false;
  ros_pc2->header = conti_input->header;
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

void MergeCallback (const conti_radar::MeasurementConstPtr& f_input,
                    const conti_radar::MeasurementConstPtr& fl_input,
                    const conti_radar::MeasurementConstPtr& fr_input,
                    const conti_radar::MeasurementConstPtr& bl_input,
                    const conti_radar::MeasurementConstPtr& br_input,
                    const nav_msgs::OdometryConstPtr& vel)
{
  std::cout << std::fixed; std::cout.precision(5);

  //cout << "f_input time: " << f_input->header.stamp.toSec() << endl;
  //cout << "vel time: " << vel->header.stamp.toSec() << endl;

  sensor_msgs::PointCloud2::Ptr f(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fl(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fr(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr bl(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr br(new sensor_msgs::PointCloud2);

  Eigen::MatrixXd ego_mat;
  ego_mat.resize(3, 1);
  ego_mat(0, 0) = vel->twist.twist.angular.z/180*M_PI;
  ego_mat(1, 0) = vel->twist.twist.linear.x;
  ego_mat(2, 0) = vel->twist.twist.linear.y;

  Eigen::MatrixXd vj_f = calcVjxVjy(ego_mat, 3.41199994087, 0);
  Eigen::MatrixXd vj_fl =calcVjxVjy(ego_mat, 2.42199993134, 0.800000011921);
  Eigen::MatrixXd vj_fr =calcVjxVjy(ego_mat, 2.42199993134, -0.800000011921);
  Eigen::MatrixXd vj_bl =calcVjxVjy(ego_mat, -0.561999976635, 0.628000020981);
  Eigen::MatrixXd vj_br =calcVjxVjy(ego_mat, -0.561999976635, -0.617999970913);

  //-- generate pointcloud from 5 radar scan --//
  conti_converter(f_input, f, vj_f, 0.003);
  conti_converter(fl_input, fl, vj_fl, 1.542);
  conti_converter(fr_input, fr, vj_fr, -1.588);
  conti_converter(bl_input, bl, vj_bl, 3.044);
  conti_converter(br_input, br, vj_br, -3.074);


  //-- td compensate pointcloud and merge them into a radar scan --//
  sensor_msgs::PointCloud2::Ptr pc;
  pc = merge(f, fl, fr, bl, br, vel);
  //pc = f;
  pc->header.stamp = ros::Time(min_t);
  pc->header.frame_id = "/car";

  //-- Publish valid_pc and invalid_pc --//
  radar_pub.publish(*pc);
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "conti_joint_doppler_weight");
  ROS_INFO("Start conti_joint_doppler_weight");
  ros::NodeHandle nh("~");

  message_filters::Subscriber<conti_radar::Measurement> sub_f(nh, "/radar_front", 1000);
  message_filters::Subscriber<conti_radar::Measurement> sub_fl(nh, "/radar_front_left", 1000);
  message_filters::Subscriber<conti_radar::Measurement> sub_fr(nh, "/radar_front_right", 1000);
  message_filters::Subscriber<conti_radar::Measurement> sub_bl(nh, "/radar_back_left", 1000);
  message_filters::Subscriber<conti_radar::Measurement> sub_br(nh, "/radar_back_right", 1000);

  //message_filters::Subscriber<nav_msgs::Odometry> sub_f_v(nh, "/f_vel_from_c", 1000);
  //message_filters::Subscriber<nav_msgs::Odometry> sub_fl_v(nh, "/fl_vel_from_c", 1000);
  //message_filters::Subscriber<nav_msgs::Odometry> sub_fr_v(nh, "/fr_vel_from_c", 1000);
  //message_filters::Subscriber<nav_msgs::Odometry> sub_bl_v(nh, "/bl_vel_from_c", 1000);
  //message_filters::Subscriber<nav_msgs::Odometry> sub_br_v(nh, "/br_vel_from_c", 1000);

  message_filters::Subscriber<nav_msgs::Odometry> sub_vel(nh, "/vel", 1000);

  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(1000),
        sub_f, sub_fl, sub_fr, sub_bl, sub_br,
        //sub_f_v, sub_fl_v, sub_fr_v, sub_bl_v, sub_br_v,
        sub_vel);

  no_cloud_sync_->registerCallback(boost::bind(&MergeCallback, _1, _2, _3, _4, _5, _6));

  radar_pub = nh.advertise<sensor_msgs::PointCloud2> ("radar_combined", 5);
  radar_weight_pub = nh.advertise<ro_msg::IntArrayStamped> ("radar_weight", 5);

  ros::spin ();
}

