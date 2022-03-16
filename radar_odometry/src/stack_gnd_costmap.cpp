#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <ro_msg/IntArrayStamped.h>
#include <ro_msg/Cov2D.h>
#include <ro_msg/Cov2DArrayStamped.h>

#include <pcl/pcl_config.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <string>
#include <thread>

#include "my_ndt_2d_cost_map.h"


using namespace std;
using namespace std::chrono_literals;

ros::Publisher pub;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry,
                                                        ro_msg::IntArrayStamped, ro_msg::Cov2DArrayStamped> NoCloudSyncPolicy;

// -- ROS param -- //
float ndt_transformation_epsilon = 0.01;
float ndt_step_size = 0.1;
float ndt_resolution = 1.0;
int ndt_iteration = 50;

int msg_flt_buffer = 100;

float ndt2d_grid_step_ = 3.0;
float ndt2d_step_size_x_ = 0.3;
float ndt2d_step_size_y_ = 0.3;
float ndt2d_step_size_theta_ = 0.5;
bool viz = false;
int ndt2d_max_it_ = 400;
double ndt2d_eps_ = 1e-6;

tf::Transform curr_tf;
bool init = false;
bool acc_init = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr old_in_pc (new pcl::PointCloud<pcl::PointXYZI>);
nav_msgs::Odometry old_ego_motion;
vector<int> old_stack_index;
vector<Eigen::Matrix2d> old_vec_cov;

double old_t = 0;
double curr_t = 0;
float old_vel = 0;
float curr_vel = 0;
float old_ang_vel = 0;
float curr_ang_vel = 0;

int index_ = 0;
int increase_grid_count = 0;

Eigen::Matrix4f getTransform(float x,float y, float theta){
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform (0,0) = std::cos (theta);
  transform (0,1) = -sin (theta);
  transform (1,0) = sin (theta);
  transform (1,1) = std::cos (theta);
  transform (0,3) = x;
  transform (1,3) = y;
  return transform;
}

double GetYawFromQ(geometry_msgs::Quaternion rot){
  tf::Quaternion q(rot.x,rot.y,rot.z,rot.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
	return yaw;
}

bool isNanInMatrix(Eigen::Matrix4f& T){
  for(int i=0; i<4; i++){
    for(int j=0; j<4; j++){
      if(T(i,j) != T(i,j) ){ // detect Nan
        return 1;
      }
    }
  }
  return 0;
}


float VelocityCal(Eigen::Matrix4f T, double delta_t){
  return -T(0,3) / delta_t;
}

float AngularVelocityCal(Eigen::Matrix4f T, double delta_t){
  float sy = sqrt(T(0,0) * T(0,0) +  T(1,0) * T(1,0) );
  bool singular = sy < 1e-6;
  float delta_yaw = 0;
  if(!singular){
    delta_yaw = atan2(T(1,0), T(0,0));
  }
  else{
    delta_yaw = 0;
  }
  return -(delta_yaw/M_PI*180) / delta_t; // in degree
}

bool CheckAcc(Eigen::Matrix4f& T){
  float curr_vel_ = VelocityCal(T, curr_t - old_t);
  //cout << "curr_vel: " << curr_vel << endl;
  float acc_ = (curr_vel_-old_vel)/(curr_t-old_t);

  float curr_ang_vel_ = AngularVelocityCal(T, curr_t - old_t);
  //cout << "curr_ang_vel: " << curr_ang_vel << endl;
  float ang_acc_ = (curr_ang_vel_-old_ang_vel)/(curr_t-old_t);

  if(!acc_init){
    acc_init = true;
    return 0;
  }
  else{
    if(curr_vel_ != curr_vel_ || curr_ang_vel_ != curr_ang_vel_){ // detect Nan
      cout << "Detect Nan \n";
      return 1;
    }
    if(abs(acc_) > 8 || abs(ang_acc_) > 150){ // 8 & 50
      cout << "=================================================================\n";
      cout << "Detect abnormal Acc \n";
      cout << "delta_t: " << curr_t - old_t << endl;
      cout << "curr_vel: " << curr_vel_ << " old_vel: " << old_vel << " acc: " << acc_ << endl;
      cout << "curr_ang_vel: " << curr_ang_vel_ << " old_ang_vel: " << old_ang_vel << " ang_acc: " << ang_acc_ << endl;

      return 1;
    }
    else{
      return 0;
    }
  }

}

Eigen::Matrix4f getInitGuess(const nav_msgs::OdometryConstPtr& ego_motion, nav_msgs::Odometry old_ego_motion){

	tf::Transform tf_ego_motion;	tf::Transform tf_old_ego_motion;
	tf::Transform tf_init_guess;

	tf::poseMsgToTF(ego_motion->pose.pose, tf_ego_motion);
	tf::poseMsgToTF(old_ego_motion.pose.pose, tf_old_ego_motion);

	tf_init_guess = tf_old_ego_motion.inverse() * tf_ego_motion;

	double delta_x = tf_init_guess.getOrigin().x();
	double delta_y = tf_init_guess.getOrigin().y();

  tf::Quaternion q = tf_init_guess.getRotation();
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

	Eigen::Matrix4f init_guess = getTransform(delta_x, delta_y, yaw);
  //cout << "Init Guess ==> delta_x: " << delta_x << " delta_y: " << delta_y << " yaw: " << yaw << endl;
	return init_guess;
}


void Vizsualization(const pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud){

    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr
    viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
    output_color (output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZI> (output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    2, "output cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem (1.0, "global");
    viewer_final->initCameraParameters ();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped ())
    {
      viewer_final->spinOnce (100);
      std::this_thread::sleep_for(100ms);
    }

}



void SeND(float grid_step_float, pcl::PointCloud<pcl::PointXYZI>::Ptr old_in_pc, vector<Eigen::Matrix2d>* old_vec_cov_ptr,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc, vector<int> stack_index, vector<Eigen::Matrix2d>* vec_cov_ptr){

  pcl::MyNormalDistributionsTransform2D<pcl::PointXYZI, pcl::PointXYZI> ndt_2d;
  //ndt_2d.setMaximumIterations (ndt2d_max_it_);
  //ndt_2d.setTransformationEpsilon (ndt2d_eps_);

  Eigen::Vector2f grid_center;	grid_center << 0, 0;
  ndt_2d.setGridCentre (grid_center);
  Eigen::Vector2f grid_extent;	grid_extent << 150, 150;
  ndt_2d.setGridExtent (grid_extent);
  Eigen::Vector2f grid_step;	grid_step << grid_step_float, grid_step_float;
  ndt_2d.setGridStep (grid_step);
  Eigen::Vector3d step_size;	step_size << ndt2d_step_size_x_, ndt2d_step_size_y_, ndt2d_step_size_theta_;
  ndt_2d.setOptimizationStepSize (step_size);

  ndt_2d.setStackIndex (stack_index);
  ndt_2d.setInputTargetCov(vec_cov_ptr);
  ndt_2d.setInputSourceCov(old_vec_cov_ptr);

  ndt_2d.setInputSource (old_in_pc); // P
  ndt_2d.setInputTarget (in_pc); // D
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  //ndt_2d.align (*output_cloud, init_guess);
  ndt_2d.align (*output_cloud);

  if(viz){
    Vizsualization(output_cloud);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
  sensor_msgs::PointCloud2 ros_pc2;
  pcl::toPCLPointCloud2(*output_cloud, *pc2);
  pcl_conversions::fromPCL(*pc2, ros_pc2);
  ros_pc2.is_dense = false;
  ros_pc2.header.frame_id = "/cost_map";
  pub.publish(ros_pc2);
}

void Callback(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& ego_motion,
              const ro_msg::IntArrayStampedConstPtr& stack_index_msg,
              const ro_msg::Cov2DArrayStampedConstPtr& pc_cov_msg){
  //cout << "---------------------------- \n";
  curr_t = input->header.stamp.toSec();

	pcl::PCLPointCloud2* in_pc2 = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*input, *in_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(*in_pc2, *in_pc);

	if (!in_pc->is_dense)
	{
		in_pc->is_dense = false;
		std::vector< int > indices;
		pcl::removeNaNFromPointCloud(*in_pc,*in_pc, indices);
	}

  vector<int> stack_index;
  for(int i=0; i<stack_index_msg->data.size(); i++){
    stack_index.push_back(stack_index_msg->data[i]);
  }
  vector<Eigen::Matrix2d> vec_cov;
  for(int i=0; i<pc_cov_msg->data.size(); i++){
    Eigen::Matrix2d cov_mat;
    cov_mat(0,0) = pc_cov_msg->data[i].covariance[0];  cov_mat(0,1) = pc_cov_msg->data[i].covariance[1];
    cov_mat(1,0) = pc_cov_msg->data[i].covariance[2];  cov_mat(1,1) = pc_cov_msg->data[i].covariance[3];
    vec_cov.push_back(cov_mat);
  }

  vector<Eigen::Matrix2d>* vec_cov_ptr (new vector<Eigen::Matrix2d>);
  vec_cov_ptr = &vec_cov;

  if(!init){ init = true;}
	else{
    SeND(ndt2d_grid_step_, old_in_pc, &old_vec_cov, in_pc, stack_index, vec_cov_ptr);
	}
	// Update
	old_in_pc = in_pc;
	old_ego_motion = *ego_motion;
  old_stack_index = stack_index;
  old_vec_cov = *vec_cov_ptr;

  old_vel = curr_vel;
  old_ang_vel = curr_ang_vel;
  old_t = curr_t;

}

int main (int argc, char** argv)
{
  //pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);

  ros::init (argc, argv, "NDT_D2D_2D_costmap");
  ros::NodeHandle nh("~");

  if (nh.getParam("msg_flt_buffer", msg_flt_buffer)) ROS_INFO("Got 'msg_flt_buffer' param: %d", msg_flt_buffer);
  else ROS_WARN("[%s] Failed to get param 'msg_flt_buffer', use default setting: %d ", ros::this_node::getName().c_str(), msg_flt_buffer);

  if (nh.getParam("viz", viz)) ROS_INFO("Got 'viz' param: %d", viz);
  else ROS_WARN("[%s] Failed to get param 'viz', use default setting: %d ", ros::this_node::getName().c_str(), viz);

	curr_tf.setIdentity();

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_stack_pc(nh, "/input_pc", 1000);
  message_filters::Subscriber<nav_msgs::Odometry> sub_ego_motion(nh, "/radar_ego_motion", 1000);
  message_filters::Subscriber<ro_msg::IntArrayStamped> sub_stack_index(nh, "/radar_stack_index", 1000);
  message_filters::Subscriber<ro_msg::Cov2DArrayStamped> sub_cov(nh, "/radar_stack_cov", 1000);

  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(msg_flt_buffer), sub_stack_pc, sub_ego_motion, sub_stack_index, sub_cov);
  no_cloud_sync_->registerCallback(boost::bind(&Callback, _1, _2, _3, _4));

  pub = nh.advertise<sensor_msgs::PointCloud2> ("ndt_cost_map", 10);

  ros::spin ();
}



