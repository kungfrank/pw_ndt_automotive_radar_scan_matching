#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/pcl_config.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>


#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <math.h>
using namespace std;
using namespace cv;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry, geometry_msgs::PointStamped> NoCloudSyncPolicy;

ros::Publisher pub;
bool init = false;
cv::Mat image_old(3000, 3000, CV_8UC1, Scalar::all(0));
nav_msgs::Odometry old_ego_motion;
tf::Transform curr_tf;

Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}

double GetYawFromQ(geometry_msgs::Quaternion rot){
	//cout <<"rot: " << rot << endl;
	if(rot.x == 0 && rot.y == 0 && rot.z == 0 && rot.w == 0){return 0;}
  tf::Quaternion q(rot.x,rot.y,rot.z,rot.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
	return yaw;
}

tf::Transform getTF(float x,float y, float theta){
	tf::Quaternion q_rot;
	q_rot.setRPY(0, 0, theta);
  tf::Vector3 origin;
  origin.setValue(x,y,static_cast<double>(0));
  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(q_rot);
  return transform;
}

cv::Mat ImageRT(cv::Mat src, double x, double y, double theta){

	cv::Mat dst;
	cv::Size dst_sz = src.size();
	cv::Mat t_mat = cv::Mat::zeros(2, 3, CV_32FC1);
/*
	t_mat.at<float>(0, 0) = cos(theta/180*M_PI);
	t_mat.at<float>(0, 1) = -sin(theta/180*M_PI);
	t_mat.at<float>(0, 2) = x * cos(theta/180*M_PI) + y * (-sin(theta/180*M_PI));
	t_mat.at<float>(1, 0) = sin(theta/180*M_PI);
	t_mat.at<float>(1, 1) = cos(theta/180*M_PI);
	t_mat.at<float>(1, 2) = x * sin(theta/180*M_PI) + y * cos(theta/180*M_PI);
	cv::warpAffine(src, dst, t_mat, dst_sz);
*/
	cout << "src.cols: " << src.cols << " src.rows: " << src.rows << endl;
	cv::Point2f center(src.cols / 2, src.rows / 2);
	double angle = theta; 
	double scale = 1; 

	t_mat = getRotationMatrix2D(center, angle, scale);
	warpAffine(src, dst, t_mat, dst_sz);

	cout << "t_mat: \n" << t_mat << endl;

	return dst;
}

cv::Mat WarpInitGuess(cv::Mat image, const nav_msgs::OdometryConstPtr& ego_motion, nav_msgs::Odometry old_ego_motion){

	cv::Mat dst;
	cv::Mat init_mat = cv::Mat::zeros(2, 3, CV_32FC1);

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

	cout << "delta_x: " << delta_x << " delta_y: " << delta_y << " yaw: " << yaw << endl;

	double cv_x = delta_y*10;
	double cv_y = delta_x*10;
	double cv_theta = -yaw;

	init_mat.at<float>(0, 0) = cos(cv_theta);
	init_mat.at<float>(0, 1) = sin(cv_theta);
	init_mat.at<float>(0, 2) = cv_x*cos(cv_theta) + cv_y*sin(cv_theta) + ( (1-cos(cv_theta))*image.cols/2 - sin(cv_theta)*image.rows/2 );
	init_mat.at<float>(1, 0) = -sin(cv_theta);
	init_mat.at<float>(1, 1) = cos(cv_theta);
	init_mat.at<float>(1, 2) = cv_x*(-sin(cv_theta)) + cv_y*cos(cv_theta) + ( sin(cv_theta)*image.cols/2 + (1-cos(cv_theta))*image.rows/2 );

	warpAffine(image, dst, init_mat, image.size());

	//cout << "image.cols/2: " << image.cols/2 << " image.rows/2: " << image.rows/2 << endl;
	//cv::Point2f center(image.cols / 2., image.cols / 2.);
	//cv::Mat rot_mat = cv::getRotationMatrix2D(center, 0.5/M_PI*180 , 1.0);
	//out << "rot_mat: \n" << rot_mat << endl;
	//cout << "init_mat: \n" << init_mat << endl;
	//warpAffine(image, dst, rot_mat, image.size());

	return dst;
}


void Callback(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& ego_motion, const geometry_msgs::PointStampedConstPtr& num_msg){
	int num = num_msg->point.x;
	cout<<"num_msg->point.x: "<<num<<endl;

	pcl::PCLPointCloud2* in_pc2 = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*input, *in_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*in_pc2, *in_pc);

	//-- remove Nan --//
	if (!in_pc->is_dense)
	{
		in_pc->is_dense = false;
		std::vector< int > indices;
		pcl::removeNaNFromPointCloud(*in_pc,*in_pc, indices);
	}

  cv::Mat image(3000, 3000, CV_8UC1, Scalar::all(0));

	vector<cv::Point2f> curr_data;
/*
	/// draw image ///
	for(int i=0 ; i<in_pc->size(); i++){ /////////////////////////// in_pc - 10_frame_num
		float dis = sqrt(pow(in_pc->points[i].x,2)+pow(in_pc->points[i].y,2));
		if(dis < 150){ // remove points far than 150m from center //
			cv::Point2f p(300-(in_pc->points[i].y + 150), 300-(in_pc->points[i].x + 150));
			cv::circle(image, cv::Point2f(p.x*10,p.y*10), 0, Scalar::all(255), FILLED);
			curr_data.push_back(p);
		}
	}
*/
	/// draw image ///

	//-- First Stage Drawing --//
  cv::Mat image1(3000, 3000, CV_8UC1, Scalar::all(0));
	for(int i=0 ; i<(in_pc->size() - num_msg->point.x); i++){
		float dis = sqrt(pow(in_pc->points[i].x,2)+pow(in_pc->points[i].y,2));
		if(dis < 150){ // remove points far than 150m from center //
			cv::Point2f p(300-(in_pc->points[i].y + 150), 300-(in_pc->points[i].x + 150));
			cv::circle(image1, cv::Point2f(p.x*10,p.y*10), 0, Scalar::all(255), FILLED);
			curr_data.push_back(p);
		}
	}
	cout << "Data Size: " << curr_data.size() << endl;
	linearPolar(image1,image1,cv::Point2f(1500, 1500), 2121.32, WARP_FILL_OUTLIERS);
	GaussianBlur( image1, image1, Size( 3 , 11 ), 1.0, 2.0 );
	linearPolar(image1,image1,cv::Point2f(1500, 1500), 2121.32, WARP_INVERSE_MAP);
	image1 = image1 * 3.0;  // Just for Visualization

	//-- Second Stage Drawing --//
  cv::Mat image2(3000, 3000, CV_8UC1, Scalar::all(0));
	for(int i=(in_pc->size() - num_msg->point.x) ; i<in_pc->size(); i++){
		float dis = sqrt(pow(in_pc->points[i].x,2)+pow(in_pc->points[i].y,2));
		if(dis < 150){ // remove points far than 150m from center //
			cv::Point2f p(300-(in_pc->points[i].y + 150), 300-(in_pc->points[i].x + 150));
			cv::circle(image2, cv::Point2f(p.x*10,p.y*10), 0, Scalar::all(255), FILLED);
			curr_data.push_back(p);
		}
	}
	cout << "Data Size: " << curr_data.size() << endl;
	linearPolar(image2,image2,cv::Point2f(1500, 1500), 2121.32, WARP_FILL_OUTLIERS);
	GaussianBlur( image2, image2, Size( 3 , 11 ), 1.0, 2.0 );
	linearPolar(image2,image2,cv::Point2f(1500, 1500), 2121.32, WARP_INVERSE_MAP);
	image2 = image2 * 3.0;  // Just for Visualization
	GaussianBlur( image2, image2, Size( 5 , 5 ), 0, 0 ); /////////////////////--- !!! Blur More !!! ---////////////////////////

	cv::addWeighted(image1, 1.0, image2, 1.0, 0.0, image);

  namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.
  imshow( "Display window", image );             // Show our image inside it.
  namedWindow( "img1", WINDOW_NORMAL );// Create a window for display.
  imshow( "img1", image1 );            // Show our image inside it.
  namedWindow( "img2", WINDOW_NORMAL );// Create a window for display.
  imshow( "img2", image2 );            // Show our image inside it.
  waitKey(0);

	if(!init){image_old = image; old_ego_motion = *ego_motion;	init = true;}

	/////////////////////////////////////////////// debug mode ///////////////////////////////////////////////
	/*
		image_old = ImageRT(image,0,0,5);
		namedWindow( "Old Image window", WINDOW_NORMAL );
		imshow( "Old Image window", image_old );
		waitKey(0);
	*/
	/////////////////////////////////////////////// debug mode end ///////////////////////////////////////////////

	//-- Apply Init Guess --//
	image_old = WarpInitGuess(image_old, ego_motion, old_ego_motion);
	//image_old = WarpInitGuess(image_old, ego_motion, old_ego_motion);
  //namedWindow( "init guess window", WINDOW_NORMAL );
	//imshow("init guess window", image);
  //namedWindow( "old window", WINDOW_NORMAL );// Create a window for display.
  //imshow( "old window", image_old );             // Show our image inside it.

	const int warp_mode = MOTION_EUCLIDEAN;
	cv::Mat warp_matrix =cv::Mat::eye(2, 3, CV_32F);
	int number_of_iterations = 500;
	// Specify the threshold of the increment
	// in the correlation coefficient between two iterations
	double termination_eps = 1e-4; //1e-5
	TermCriteria criteria (TermCriteria::COUNT+TermCriteria::EPS, number_of_iterations, termination_eps);
	findTransformECC(image_old, image, warp_matrix, warp_mode, criteria);

	Mat image_aligned;
	 
	if (warp_mode != MOTION_HOMOGRAPHY)
		  // Use warpAffine for Translation, Euclidean and Affine
		  warpAffine(image, image_aligned, warp_matrix, image_old.size(), INTER_LINEAR + WARP_INVERSE_MAP);
	else
		  // Use warpPerspective for Homography
		  warpPerspective (image, image_aligned, warp_matrix, image_old.size(),INTER_LINEAR + WARP_INVERSE_MAP);

	// Show final result
  namedWindow( "Aligned window", WINDOW_NORMAL );
	imshow("Aligned window", image_aligned);
	cout << "warp_matrix: \n" << warp_matrix << endl;

	Mat R_mat;
	R_mat = cv::Mat::eye(3, 3, CV_64F);
	R_mat.at<double>(0,0) = warp_matrix.at<float>(0,0);
	R_mat.at<double>(0,1) = warp_matrix.at<float>(0,1);
	R_mat.at<double>(1,0) = warp_matrix.at<float>(1,0);
	R_mat.at<double>(1,1) = warp_matrix.at<float>(1,1);

	Mat T_mat;
	T_mat = cv::Mat::zeros(3, 1, CV_64F);
	T_mat.at<double>(0,0) = warp_matrix.at<float>(0,2);
	T_mat.at<double>(1,0) = warp_matrix.at<float>(1,2);
	T_mat.at<double>(2,0) = 0;

	Vec3f rotation_vec = rotationMatrixToEulerAngles(R_mat);

	Mat center_rotation_effect_mat;
	center_rotation_effect_mat = cv::Mat::zeros(3, 1, CV_64F);
	center_rotation_effect_mat.at<double>(0,0) = (1-cos(rotation_vec[2])) * image.cols/2 + sin(rotation_vec[2]) * image.rows/2;
	center_rotation_effect_mat.at<double>(1,0) = -sin(rotation_vec[2]) * image.cols/2 + (1-cos(rotation_vec[2])) * image.rows/2;
	center_rotation_effect_mat.at<double>(2,0) = 0;

	Mat translation_vec;
	translation_vec = cv::Mat::zeros(3, 1, CV_64F);
	translation_vec = R_mat.inv() * (T_mat - center_rotation_effect_mat);

	double trans_x = translation_vec.at<double>(1,0)/10;
	double trans_y = translation_vec.at<double>(0,0)/10;

	//-- ECC result (after init guess) --//
	cout << "X: " << trans_x << " m" << endl;
	cout << "Y: " << trans_y << " m" << endl;
	cout << "Theta: " << rotation_vec[2]/M_PI*180 << " degree" << endl;

	//-- combine init guess & ECC result --//
	tf::Transform tf_ego_motion;	tf::Transform tf_old_ego_motion;
	tf::Transform tf_init_guess;
	tf::poseMsgToTF(ego_motion->pose.pose, tf_ego_motion);
	tf::poseMsgToTF(old_ego_motion.pose.pose, tf_old_ego_motion);
	tf_init_guess = tf_old_ego_motion.inverse() * tf_ego_motion;
	//cout << "ego_motion->pose.pose.position.x: " << ego_motion->pose.pose.position.x << endl;
	//cout << "old_ego_motion.pose.pose.position.x: " << old_ego_motion.pose.pose.position.x << endl;

	tf::Transform ecc_tf = getTF(trans_x, trans_y, rotation_vec[2]);

  tf::Transform final_tf = ecc_tf*tf_init_guess; // * ecc_tf

	waitKey(10);
//////////////////////////////////////////////////

	image_old = image;
	old_ego_motion = *ego_motion;
  curr_tf = curr_tf * final_tf;

	nav_msgs::Odometry odom;
	odom.header.stamp = input->header.stamp;
	odom.header.frame_id = "/origin";
	odom.child_frame_id = "/origin";
	tf::poseTFToMsg(curr_tf, odom.pose.pose);
	pub.publish(odom);

	cout << endl;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "radar_scan_to_img");
  ros::NodeHandle n("~");

  //ros::Subscriber sub = n.subscribe("/input_pc", 1000, Callback);

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_stack_pc(n, "/input_pc", 200);
  message_filters::Subscriber<nav_msgs::Odometry> sub_ego_motion(n, "/radar_ego_motion", 200);

  message_filters::Subscriber<geometry_msgs::PointStamped> sub_num(n, "/radar_point_num_after_10", 200);
  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(1000), sub_stack_pc, sub_ego_motion, sub_num);
  no_cloud_sync_->registerCallback(boost::bind(&Callback, _1, _2, _3));

	pub = n.advertise<nav_msgs::Odometry> ("img_match_odom", 1);
	curr_tf.setIdentity();
  ros::spin();

  return 0;

}

/*
  pcl::PCLPointCloud2* in_cloud = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*input, *in_cloud);
  pcl::PointCloud<pcl::PointXYZ> inc;
  pcl::fromPCLPointCloud2(*in_cloud,inc);

  pcl::PCLPointCloud2* out_cloud = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(output, *out_cloud);
  pcl::PointCloud<pcl::PointXYZ> outc;
  pcl::fromPCLPointCloud2(*out_cloud,outc);

  outc = outc + inc;

  pcl::toPCLPointCloud2(outc, out_cloud);

  pcl_conversions::fromPCL(*out_cloud, output);
*/

