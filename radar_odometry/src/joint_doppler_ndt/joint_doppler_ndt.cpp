/********************************************************
This is the implementation of the probabilistic radar ego-motion on Nuscenes dataset following paper "Probabilistic ego-motion estimation using multiple automotive radar sensors", in 2017 RAS
-------------------------------
INPUT Topic:
  Velocity: (nav_msgs::Odometry)

-------------------------------
OUTPUT Topic: (nav_msgs::Odometry)
  /joint_doppler_ndt_odom

-------------------------------
by Frank Kung 2020 Aug
*********************************************************/

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/pcl_config.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
//#include <pcl/registration/ndt.h>
//#include <pcl/registration/ndt_2d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <iostream>


#include <thread>
#include <pcl/visualization/pcl_visualizer.h>

#include "weighted_ndt_2d.h"

using namespace std;
using namespace std::chrono_literals;

ros::Publisher pub;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> NoCloudSyncPolicy;

// -- ROS param -- //
float ndt_transformation_epsilon = 0.01;
float ndt_step_size = 0.1;
float ndt_resolution = 1.0;
int ndt_iteration = 50;
////////////////////
float ndt2d_grid_step = 3.0;
float ndt2d_step_size = 0.01;
float ndt2d_transformation_epsilon = 0.0001;
int ndt2d_iteration = 50;

int msg_flt_buffer = 100;
bool viz = false;

tf::Transform curr_tf;
bool init = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr old_in_pc (new pcl::PointCloud<pcl::PointXYZI>);
nav_msgs::Odometry old_ego_motion;

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

Eigen::Matrix4f weightedNDT2d(pcl::PointCloud<pcl::PointXYZI>::Ptr old_in_pc, pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc,
                      Eigen::Matrix4f init_guess, float grid_step_float){
  //cout << "old_in_pc->size(): " << old_in_pc->size() << endl;
  //cout << "in_pc->size(): " << in_pc->size() << endl;

  // Initializing Normal Distributions Transform (NDT2D).

  pcl::WeightedNormalDistributionsTransform2D<pcl::PointXYZI, pcl::PointXYZI> w_ndt_2d;

  w_ndt_2d.setMaximumIterations (ndt2d_iteration);
  w_ndt_2d.setTransformationEpsilon (ndt2d_transformation_epsilon);
  Eigen::Vector2f grid_center;	grid_center << 0, 0;
  w_ndt_2d.setGridCentre (grid_center);
  Eigen::Vector2f grid_extent;	grid_extent << 150, 150;
  w_ndt_2d.setGridExtent (grid_extent);
  Eigen::Vector2f grid_step;	grid_step << grid_step_float, grid_step_float;
  w_ndt_2d.setGridStep (grid_step);
  Eigen::Vector3d step_size;	step_size << ndt2d_step_size, ndt2d_step_size, ndt2d_step_size; //0.05, 0.01, 0.01;
  w_ndt_2d.setOptimizationStepSize (step_size);
  w_ndt_2d.setInputSource (old_in_pc);
  w_ndt_2d.setInputTarget (in_pc);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  w_ndt_2d.align (*output_cloud, init_guess.inverse());
  //ndt_2d.align (*output_cloud);
  std::cout << "2D NDT has converged: " << w_ndt_2d.hasConverged ();

  Eigen::Matrix4f Tm;
  Tm = w_ndt_2d.getFinalTransformation();

  if( !w_ndt_2d.hasConverged() || isNanInMatrix(Tm)){ //|| w_ndt_2d.getFitnessScore () > 100
    cout << " NDT2D Fail !! try increasing size... \n";
    Tm = weightedNDT2d(old_in_pc, in_pc, init_guess, grid_step_float + 2.5); // increasing grid size
  }
  else{
    cout << " score: " << w_ndt_2d.getFitnessScore () << std::endl;

    ///--- Visualization ---///
    if(viz){
      // Initializing point cloud visualizer
      pcl::visualization::PCLVisualizer::Ptr
      viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer_final->setBackgroundColor (0, 0, 0);
      // Coloring and visualizing target cloud (red).
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
      target_color (in_pc, 255, 0, 0);
      viewer_final->addPointCloud<pcl::PointXYZI> (in_pc, target_color, "target cloud");
      viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                      5, "target cloud");
      // Coloring and visualizing transformed input cloud (green).
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
      output_color (output_cloud, 0, 255, 0);
      viewer_final->addPointCloud<pcl::PointXYZI> (output_cloud, output_color, "output cloud");
      viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                      5, "output cloud");

      // Coloring and visualizing transformed input cloud (blue).
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
      input_color (old_in_pc, 0, 100, 255);
      viewer_final->addPointCloud<pcl::PointXYZI> (old_in_pc, input_color, "input cloud");
      viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                      5, "input cloud");
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
  }
  return Tm;

}

void Callback(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& ego_motion){

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

  if(!init){init = true;}
  else{

    Eigen::Matrix4f init_guess = getInitGuess(ego_motion, old_ego_motion);
    Eigen::Matrix4f Tm = weightedNDT2d(old_in_pc, in_pc, init_guess, ndt2d_grid_step);

    init_guess = getTransform(0,0,0);

    tf::Vector3 origin;
    origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));
    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
          static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
          static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));
/*
    std::cout << "-Tm(0,3): " << -Tm(0,3)
              << " -Tm(1,3): " << -Tm(1,3) << std::endl;
*/
    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);
    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(tfqt);

    curr_tf = transform * curr_tf;

    nav_msgs::Odometry odom;
    odom.header.stamp = input->header.stamp;
    odom.header.frame_id = "/origin";
    odom.child_frame_id = "/joint_doppler_ndt_odom";
    tf::poseTFToMsg(curr_tf.inverse(), odom.pose.pose);
    //odom.twist.twist.linear.x = ndt.getFitnessScore(); // odom.twist.twist.linear.x --> NDT score
    pub.publish(odom);
  }

  // Update
  old_in_pc = in_pc;
  old_ego_motion = *ego_motion;

}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "Weighted NDT");
  ros::NodeHandle nh("~");

  if (nh.getParam("ndt2d_grid_step", ndt2d_grid_step)) ROS_INFO("Got 'ndt2d_grid_step' param: %.2f", ndt2d_grid_step);
  else ROS_WARN("[%s] Failed to get param 'ndt2d_grid_step', use default setting: %.2f ", ros::this_node::getName().c_str(), ndt2d_grid_step);

  if (nh.getParam("ndt2d_step_size", ndt2d_step_size)) ROS_INFO("Got 'ndt2d_step_size' param: %.2f", ndt2d_step_size);
  else ROS_WARN("[%s] Failed to get param 'ndt2d_step_size', use default setting: %.2f ", ros::this_node::getName().c_str(), ndt2d_step_size);

  if (nh.getParam("ndt2d_transformation_epsilon", ndt2d_transformation_epsilon)) ROS_INFO("Got 'ndt2d_transformation_epsilon' param: %.2f", ndt2d_transformation_epsilon);
  else ROS_WARN("[%s] Failed to get param 'ndt2d_transformation_epsilon', use default setting: %.2f", ros::this_node::getName().c_str(), ndt2d_transformation_epsilon);

  if (nh.getParam("ndt2d_iteration", ndt2d_iteration)) ROS_INFO("Got 'ndt2d_iteration' param: %d", ndt2d_iteration);
  else ROS_WARN("[%s] Failed to get param 'ndt2d_iteration', use default setting: %d ", ros::this_node::getName().c_str(), ndt2d_iteration);

  if (nh.getParam("msg_flt_buffer", msg_flt_buffer)) ROS_INFO("Got 'msg_flt_buffer' param: %d", msg_flt_buffer);
  else ROS_WARN("[%s] Failed to get param 'msg_flt_buffer', use default setting: %d ", ros::this_node::getName().c_str(), msg_flt_buffer);

  if (nh.getParam("viz", viz)) ROS_INFO("Got 'viz' param: %d", viz);
  else ROS_WARN("[%s] Failed to get param 'viz', use default setting: %d ", ros::this_node::getName().c_str(), viz);

  curr_tf.setIdentity();

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc(nh, "/input_pc", 2000);
  message_filters::Subscriber<nav_msgs::Odometry> sub_ego_motion(nh, "/radar_ego_motion", 2000);

  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(msg_flt_buffer), sub_pc, sub_ego_motion);
  no_cloud_sync_->registerCallback(boost::bind(&Callback, _1, _2));

  pub = nh.advertise<nav_msgs::Odometry> ("joint_doppler_ndt_odom", 200);


  ros::spin ();
}



