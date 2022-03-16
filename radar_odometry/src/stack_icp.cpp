/********************************************************
This is the node for ICP.
-------------------------------
INPUT Topic:
  PointCloud: (sensor_msgs::PointCloud2)
		/input_pc
  Velocity integration: (nav_msgs::Odometry)
		/radar_ego_motion
-------------------------------
OUTPUT Topic:
	Odomtery: (nav_msgs::Odometry)
		icp_odom

by Frank Kung 2019 Dec
*********************************************************/

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/pcl_config.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

#include "cov_func_point_to_point.h"
#include "icp_cov.h"

using namespace std;

ros::Publisher pub;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> NoCloudSyncPolicy;

// -- ROS param -- //
float icp_max_dis = 10;
int icp_iteration = 100;
int msg_flt_buffer = 100;

bool use_init_guess = 1;

tf::Transform curr_tf;
bool init = false;

pcl::PointCloud<pcl::PointXYZ>::Ptr old_in_pc (new pcl::PointCloud<pcl::PointXYZ>);
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

tf::Transform getTF(float x, float y, float theta){
	tf::Quaternion q_rot;
	q_rot.setRPY(0, 0, theta);
  tf::Vector3 origin;
  origin.setValue(x,y,static_cast<double>(0));
  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(q_rot);
  return transform;
}

double GetYawFromQ(geometry_msgs::Quaternion rot){
  tf::Quaternion q(rot.x,rot.y,rot.z,rot.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
	return yaw;
}

/*
Eigen::Matrix4f getInitGuess(const nav_msgs::OdometryConstPtr& ego_motion, nav_msgs::Odometry old_ego_motion){

	double delta_x = ego_motion->pose.pose.position.x - old_ego_motion.pose.pose.position.x;
	double delta_y = ego_motion->pose.pose.position.y - old_ego_motion.pose.pose.position.y;
	double delta_theta = GetYawFromQ(ego_motion->pose.pose.orientation) - GetYawFromQ(old_ego_motion.pose.pose.orientation);
	Eigen::Matrix4f init_guess = getTransform(delta_x, delta_y, delta_theta);

	cout << "delta_x: " << delta_x << " delta_y: " << delta_y << " delta_theta: " << delta_theta << endl;

	return init_guess;
}
*/

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
    //cout << "delta_x: " << delta_x << " delta_y: " << delta_y << " yaw: " << yaw << endl;
    return init_guess;
}


void Callback(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& ego_motion){

        pcl::PCLPointCloud2* in_pc2 = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*input, *in_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*in_pc2, *in_pc);

	if (!in_pc->is_dense)
	{
		in_pc->is_dense = false;
		std::vector< int > indices;
		pcl::removeNaNFromPointCloud(*in_pc,*in_pc, indices);
	}

  //cout << "pc size: " << in_pc->size() << endl;

	if(!init){init = true;}
	else{
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		// Set the max correspondence distance to _m (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (icp_max_dis);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations (icp_iteration);
		icp.setTransformationEpsilon(1e-6); //1e-14 
		icp.setEuclideanFitnessEpsilon(1e-6); //1e-4
		
		icp.setInputSource(old_in_pc);
		icp.setInputTarget(in_pc);
		pcl::PointCloud<pcl::PointXYZ> Final;
		Eigen::Matrix4f init_guess = getInitGuess(ego_motion, old_ego_motion);
		//init_guess = getTransform(10,10,3);

    if(use_init_guess){
      icp.align(Final, init_guess.inverse());
    }
    else{
      icp.align(Final);
    }

		init_guess = getTransform(0,0,0);

		//std::cout << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

		Eigen::Matrix4f Tm = icp.getFinalTransformation();

    //std::cout << "icp_x: " << -Tm(0,3) << " icp_y: " << -Tm(1,3) << std::endl;

		tf::Vector3 origin;
		origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));
		tf::Matrix3x3 tf3d;
		tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)), 
		      static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)), 
		      static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));
		tf::Quaternion tfqt;
		tf3d.getRotation(tfqt);
		tf::Transform transform;
		transform.setOrigin(origin);
		transform.setRotation(tfqt);

    curr_tf = transform * curr_tf;

		nav_msgs::Odometry odom;
		odom.header.stamp = input->header.stamp;
    odom.header.frame_id = "/ro_origin";
    odom.child_frame_id = "/ro_origin";
		tf::poseTFToMsg(curr_tf.inverse(), odom.pose.pose);
		odom.twist.twist.linear.x = icp.getFitnessScore(); // odom.twist.twist.linear.x --> ICP score
		pub.publish(odom);

		//std::cout << "x: " << odom.pose.pose.position.x << " y: " << odom.pose.pose.position.y << std::endl;
/*
		//-- Caculate ICP Cov --//
		pcl::PointCloud<pcl::PointXYZ>::Ptr old_in_pc_align (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*old_in_pc, *old_in_pc_align, Tm);

    pcl::Correspondences correspondeces_reciprocal_shot;
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;

    //corr_est.setInputSource(old_in_pc);
		corr_est.setInputSource(old_in_pc_align);

    corr_est.setInputTarget(in_pc);
    corr_est.determineReciprocalCorrespondences(correspondeces_reciprocal_shot);
		cout << "No. of Reciprocal Correspondences : " << correspondeces_reciprocal_shot.size() << endl;

    std::vector<int> data_idx;
    std::vector<int> model_idx;

    pcl::Correspondence temp1;
    for (int i = 0; i < correspondeces_reciprocal_shot.size(); i++)
    {
        temp1 = correspondeces_reciprocal_shot[i];
        data_idx.push_back(temp1.index_query);
        model_idx.push_back(temp1.index_match);
    }

    pcl::PointCloud<pcl::PointXYZ> data_pi; //Put all the pi in this cloud and its size will be equal to number of correspondences
    pcl::PointCloud<pcl::PointXYZ> model_qi;// Put all the qi in this cloud and its size will be equal to number of correspondences

    pcl::copyPointCloud(*old_in_pc_align,data_idx,data_pi);
    pcl::copyPointCloud(*in_pc,model_idx,model_qi);

    Eigen::MatrixXd ICP_COV_(6,6);
    ICP_COV_ = Eigen::MatrixXd::Zero(6,6);
    calculate_ICP_COV(data_pi, model_qi, Tm, ICP_COV_);
		//-- Caculate ICP Cov End --//

		//-- Caculate ICP Cov by Hessian --//
    Eigen::MatrixXd cov(6,6);
    cov = Eigen::MatrixXd::Zero(6,6);
		ICP_COV(old_in_pc, in_pc, Tm, cov);
		//-- Caculate ICP Cov by Hessian --//
*/
	}

	// Update
  old_in_pc = in_pc;
	old_ego_motion = *ego_motion;
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "ICP");
    ros::NodeHandle nh("~");

    if (nh.getParam("icp_max_dis", icp_max_dis)) ROS_INFO("Got 'icp_max_dis' param: %.2f", icp_max_dis);
    else ROS_WARN("[%s] Failed to get param 'icp_max_dis', use default setting: %.2f", ros::this_node::getName().c_str(), icp_max_dis);

    if (nh.getParam("icp_iteration", icp_iteration)) ROS_INFO("Got 'icp_iteration' param: %d", icp_iteration);
    else ROS_WARN("[%s] Failed to get param 'icp_iteration', use default setting: %d ", ros::this_node::getName().c_str(), icp_iteration);

    if (nh.getParam("use_init_guess", use_init_guess)) ROS_INFO("Got 'use_init_guess' param: %d", use_init_guess);
    else ROS_WARN("[%s] Failed to get param 'use_init_guess', use default setting: %d ", ros::this_node::getName().c_str(), use_init_guess);

    if (nh.getParam("msg_flt_buffer", msg_flt_buffer)) ROS_INFO("Got 'msg_flt_buffer' param: %d", msg_flt_buffer);
    else ROS_WARN("[%s] Failed to get param 'msg_flt_buffer', use default setting: %d ", ros::this_node::getName().c_str(), msg_flt_buffer);

        curr_tf.setIdentity();

    //ros::Subscriber sub = nh.subscribe ("/input_pc", 2, icp_cb);
    //ros::Subscriber sub_vel = nh.subscribe ("/vel", 2, vel_cb);

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_stack_pc(nh, "/input_pc", 50);
    message_filters::Subscriber<nav_msgs::Odometry> sub_ego_motion(nh, "/radar_ego_motion", 200);

    message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

    no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(msg_flt_buffer), sub_stack_pc, sub_ego_motion);
    no_cloud_sync_->registerCallback(boost::bind(&Callback, _1, _2));

    pub = nh.advertise<nav_msgs::Odometry> ("icp_odom", 1);


  ros::spin ();
}



