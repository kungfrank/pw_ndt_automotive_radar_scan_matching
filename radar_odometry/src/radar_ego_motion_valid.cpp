#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <conti_radar/Measurement.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <thread>
#include <math.h>
#include <algorithm>
#include <string>
using namespace std;
using namespace std::literals::chrono_literals;

ros::Publisher pub;

ros::Publisher f_outlier_pub;
ros::Publisher fr_outlier_pub;
ros::Publisher fl_outlier_pub;
ros::Publisher br_outlier_pub;
ros::Publisher bl_outlier_pub;

ros::Publisher f_inlier_pub;
ros::Publisher fr_inlier_pub;
ros::Publisher fl_inlier_pub;
ros::Publisher br_inlier_pub;
ros::Publisher bl_inlier_pub;

ros::Publisher f_vel_pub;
ros::Publisher fr_vel_pub;
ros::Publisher fl_vel_pub;
ros::Publisher br_vel_pub;
ros::Publisher bl_vel_pub;

ros::Publisher f_vel_c_pub;
ros::Publisher fr_vel_c_pub;
ros::Publisher fl_vel_c_pub;
ros::Publisher br_vel_c_pub;
ros::Publisher bl_vel_c_pub;

typedef message_filters::sync_policies::ApproximateTime<conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        conti_radar::Measurement,
																												conti_radar::Measurement> NoCloudSyncPolicy;

bool comp(int a, int b) 
{ 
    return (a < b); 
} 

pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

std::vector<int> calcRansacMat(Eigen::MatrixXd VD_mat, Eigen::MatrixXd R_mat, double threshold) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = R_mat.rows();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = R_mat(i, 0); //1
        cloud->points[i].y = R_mat(i, 1); //2
        cloud->points[i].z = VD_mat(i, 0);
    }

    std::vector<int> inliers;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

    // Do RANSAC
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setMaxIterations(2000);
    ransac.setDistanceThreshold(threshold); //0.05 in low mode 0.005 in highway mode
    ransac.computeModel();
    ransac.getInliers(inliers);

	// viz //
	/*
  	pcl::copyPointCloud (*cloud, inliers, *final);

		pcl::visualization::PCLVisualizer::Ptr viewer;
		viewer = simpleVis(cloud);
		while (!viewer->wasStopped ())
		{
			viewer->spinOnce (100);
			std::this_thread::sleep_for(100ms);
		}
	*/
	// viz end //

    return inliers;
}

Eigen::MatrixXd deleteCols(Eigen::MatrixXd mat, std::vector<int> cols) {
    Eigen::MatrixXd rm_mat(cols.size(), mat.cols());
    int current_row = 0, current_vec = 0;
    for (int i = 0; i < rm_mat.rows(); i++)
        rm_mat.row(i) = mat.row(cols[i]);
    return rm_mat;
}

void radar_sensor(const conti_radar::MeasurementConstPtr& input, float beta, float r_x, float r_y, Eigen::MatrixXd& M_mat , Eigen::MatrixXd& R_mat, Eigen::MatrixXd& VD_mat){
	double valid_size = 0;
	for(int i=0;i<(input->points.size());i++)if ((input->points[i].invalid_state) == 0x00){valid_size++;}
 // || (input->points[i].invalid_state) == 0x04 || (input->points[i].invalid_state) == 0x0b
	//valid_size = input->points.size();
	cout << "size: " << valid_size << endl;

	VD_mat.resize(valid_size,1); // N x 1
	M_mat.resize(valid_size, 2); // N x 2
	R_mat.resize(valid_size, 3); // N x 3

	int n=0;
	for(int i=0; i< (input->points.size()); i++){
		if ((input->points[i].invalid_state) == 0x00){
			double theta;
			double x = input->points[i].longitude_dist;
			double y = input->points[i].lateral_dist;
			double vx = input->points[i].longitude_vel;
			double vy = input->points[i].lateral_vel;
			theta = atan2(y, x);
			theta = theta + beta;
			M_mat(n,0) = cos(theta);
			M_mat(n,1) = sin(theta);

			Eigen::Vector2d radial_vector(-x, -y);
			Eigen::Vector2d radial_vector_normalized = radial_vector.normalized();
			Eigen::Vector2d vel(vx, vy);
			double vel_radial = vel.dot(radial_vector_normalized); //projection

			VD_mat(n,0) = vel_radial;
			n++;
		}
	}

	Eigen::MatrixXd S_mat;
	//-- for general condition (omega,vx,vy)--//
	S_mat.resize(2, 3); // 2 x 3
	S_mat(0,0) = -r_y; S_mat(0,1) = 1.0; S_mat(0,2) = 0.0;
	S_mat(1,0) = r_x;  S_mat(1,1) = 0.0; S_mat(1,2) = 1.0;
	R_mat = M_mat*S_mat; // N x 3
/*
	//-- for ackerman condition (omega,vx)--//
	S_mat.resize(2, 2); // 2 x 2
	S_mat(0,0) = -r_y; S_mat(0,1) = 1.0;
	S_mat(1,0) = r_x;  S_mat(1,1) = 0.0;
	R_mat = M_mat*S_mat; // N x 2
*/
}

conti_radar::Measurement contiMsgOutlierFilter(const conti_radar::MeasurementConstPtr& input, vector<int> outliers){
	conti_radar::Measurement conti_msg_array;
	conti_msg_array.header = input->header;
	if(outliers.size()>0){
		int j=0;
		for(int i=0; i< (input->points.size()); i++){
			if(i == outliers[j]){
				conti_msg_array.points.push_back(input->points[i]);
				j++;
			}
		}
	}
	return conti_msg_array;
}

conti_radar::Measurement contiMsgOutlierFilterComp(const conti_radar::MeasurementConstPtr& input, vector<int> outliers, float beta, Eigen::MatrixXd vj){

	conti_radar::Measurement conti_msg_array;
	conti_radar::ContiRadar point_msg;
	conti_msg_array.header = input->header;

	tf::Transform radar_to_center;
	tf::Quaternion q; q.setRPY(0, 0, -beta);
  radar_to_center.setRotation(q);

	tf::Transform radar_v_from_center;
	radar_v_from_center.setOrigin( tf::Vector3(vj(0,0), vj(1,0), 0) );

	tf::Transform radar_v_from_radar = radar_to_center * radar_v_from_center;

	if(outliers.size()>0){
		int j=0;
		for(int i=0; i< (input->points.size()); i++){
			if(i == outliers[j]){
				point_msg = input->points[i];
				point_msg.longitude_vel = input->points[i].longitude_vel + radar_v_from_radar.getOrigin().x(); // vx
				point_msg.lateral_vel = input->points[i].lateral_vel + radar_v_from_radar.getOrigin().y(); // vy
				conti_msg_array.points.push_back(point_msg);
				j++;
			}
		}
	}
	return conti_msg_array;
}

conti_radar::Measurement contiMsgInlierFilter(const conti_radar::MeasurementConstPtr& input, vector<int> inliers){
	conti_radar::Measurement conti_msg_array;
	conti_msg_array.header = input->header;
	if(inliers.size()>0){
		int j=0;
		for(int i=0; i< (input->points.size()); i++){
			if(i == inliers[j]){
				conti_msg_array.points.push_back(input->points[i]);
				j++;
			}
		}
	}
	return conti_msg_array;
}

vector<int> outlierToInlier(vector<int> outlier, int size) {
	vector<int> inlier;
	if(outlier.size()>0){
		int j=0;
		for(int i=0;i<size;i++){
			if(i == outlier[j]){ j++;}
			else{ inlier.push_back(i);}
		}
	}
	else
		for(int i=0;i<size;i++) inlier.push_back(i);
  return inlier;
}

Eigen::MatrixXd calcVj(Eigen::MatrixXd A, Eigen::MatrixXd y){ // y = Ax
	Eigen::MatrixXd x;
	x.resize(2,1);
	if(A.rows()>2 && y.rows()>2){
		x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);
		//cout << "solution: " << x(0,0) << ", " << x(1,0) << endl;
	}
	return x;
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


void Callback (const conti_radar::MeasurementConstPtr& f_input,
               const conti_radar::MeasurementConstPtr& fl_input,
               const conti_radar::MeasurementConstPtr& fr_input,
               const conti_radar::MeasurementConstPtr& bl_input,
               const conti_radar::MeasurementConstPtr& br_input) 
{
	double f_t = f_input->header.stamp.toSec();
	double fl_t = fl_input->header.stamp.toSec();
	double fr_t = fr_input->header.stamp.toSec();
	double bl_t = bl_input->header.stamp.toSec();
	double br_t = br_input->header.stamp.toSec();
	double list[] = {f_t,fl_t,fr_t,bl_t,br_t};

	std::cout << std::fixed; std::cout.precision(5);
/*
  cout << "front time: " << f_t << endl;
  cout << "fl time: " << fl_t << endl;
  cout << "fr time: " << fr_t << endl;
  cout << "bl time: " << bl_t << endl;
  cout << "br time: " << br_t << endl;
*/
	double min_t = *std::min_element(list,list+5);
  cout << "biggest time difference: " << (*std::max_element(list,list+5)-*std::min_element(list,list+5)) << endl;
	cout << endl;

	Eigen::MatrixXd M_mat_f;
	Eigen::MatrixXd R_mat_f;
	Eigen::MatrixXd VD_mat_f;

	float f_x = 3.41199994087; //3.6
	float f_y = 0.0;
	float beta_f = 0.003;
	radar_sensor(f_input, beta_f, f_x, f_y, M_mat_f, R_mat_f, VD_mat_f);

	Eigen::MatrixXd M_mat_fr;
	Eigen::MatrixXd R_mat_fr;
	Eigen::MatrixXd VD_mat_fr;
	float fr_x = 2.42199993134;
	float fr_y = -0.800000011921;
	float beta_fr = -1.588;
	radar_sensor(fr_input, beta_fr, fr_x, fr_y, M_mat_fr, R_mat_fr, VD_mat_fr);

	Eigen::MatrixXd M_mat_fl;
	Eigen::MatrixXd R_mat_fl;
	Eigen::MatrixXd VD_mat_fl;
	float fl_x = 2.42199993134;
	float fl_y = 0.800000011921;
	float beta_fl = 1.542;
	radar_sensor(fl_input, beta_fl, fl_x, fl_y, M_mat_fl, R_mat_fl, VD_mat_fl);

	Eigen::MatrixXd M_mat_br;
	Eigen::MatrixXd R_mat_br;
	Eigen::MatrixXd VD_mat_br;
	float br_x = -0.561999976635;
	float br_y = -0.617999970913;
	float beta_br = -3.074;
	radar_sensor(br_input, beta_br, br_x, br_y, M_mat_br, R_mat_br, VD_mat_br);

	Eigen::MatrixXd M_mat_bl;
	Eigen::MatrixXd R_mat_bl;
	Eigen::MatrixXd VD_mat_bl;
	float bl_x = -0.561999976635;
	float bl_y = 0.628000020981;
	float beta_bl = 3.044;
	radar_sensor(bl_input, beta_bl, bl_x, bl_y, M_mat_bl, R_mat_bl, VD_mat_bl);

	int f_size = R_mat_f.rows(); int fr_size = R_mat_fr.rows(); int fl_size = R_mat_fl.rows();
	int br_size = R_mat_br.rows(); int bl_size = R_mat_bl.rows();

	Eigen::MatrixXd R(f_size+fr_size+fl_size+br_size+bl_size, R_mat_f.cols());
	R << R_mat_f, R_mat_fr, R_mat_fl, R_mat_br, R_mat_bl;
	Eigen::MatrixXd VD(f_size+fr_size+fl_size+br_size+bl_size, VD_mat_f.cols());
	VD << VD_mat_f, VD_mat_fr, VD_mat_fl, VD_mat_br, VD_mat_bl;

/*
	//-- RANSAC for only front radar --//
	std::vector<int> f_inliers;
	f_inliers = calcRansacMat(VD_mat_f, R_mat_f, 0.1);
  Eigen::MatrixXd RANSAC_R_mat_f = deleteCols(R_mat_f, f_inliers);
  Eigen::MatrixXd RANSAC_VD_mat_f = deleteCols(VD_mat_f, f_inliers);
	cout << "R_mat_f.rows(): " << R_mat_f.rows() << endl;
	cout << "RANSAC_R_mat_f.rows(): " << RANSAC_R_mat_f.rows() << endl;
	//-- End RANSAC --//
	Eigen::MatrixXd ego_mat = RANSAC_R_mat_f.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(RANSAC_VD_mat_f);
*/

	//-- RANSAC --//
	std::vector<int> inliers;
	inliers = calcRansacMat(VD, R, 0.3);   //0.3
  Eigen::MatrixXd RANSAC_R = deleteCols(R, inliers);
  Eigen::MatrixXd RANSAC_VD = deleteCols(VD, inliers);
	cout << "R.rows(): " << R.rows() << endl;
	cout << "RANSAC_R.rows(): " << RANSAC_R.rows() << endl;
	//-- End RANSAC --//

	//-- Vel Msg--//
	Eigen::MatrixXd ego_mat = RANSAC_R.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(RANSAC_VD);

	cout << "w: " << ego_mat(0,0)/M_PI*180 << endl;
	cout << "vx: " << ego_mat(1,0) << endl;
	cout << "vy: " << ego_mat(2,0) << endl;
	cout << endl;

	nav_msgs::Odometry odom;
  odom.header.frame_id = "/car";
  odom.header.stamp = ros::Time(min_t);
  odom.twist.twist.linear.x = ego_mat(1,0);
  odom.twist.twist.linear.y = ego_mat(2,0);
  odom.twist.twist.angular.z = ego_mat(0,0)/M_PI*180;
	//-- Vel Msg End --//

	//-- Outliers Msg (comp) --//
	std::vector<int> f_outliers, fr_outliers, fl_outliers, br_outliers, bl_outliers;

	int j=0;
	for(int i=0; i<R.rows(); i++){
		if(inliers[j] == i){j++;}
		else{
			if(i< f_size){ f_outliers.push_back(i); }
			else if(f_size <= i && i < (f_size+fr_size)){ fr_outliers.push_back(i-(f_size)); }
			else if((f_size+fr_size) <= i && i < (f_size+fr_size+fl_size)){ fl_outliers.push_back(i-(f_size+fr_size)); }
			else if((f_size+fr_size+fl_size) <= i && i < (f_size+fr_size+fl_size+br_size)){ br_outliers.push_back(i-(f_size+fr_size+fl_size)); }
			else if((f_size+fr_size+fl_size+br_size) <= i){ bl_outliers.push_back(i-(f_size+fr_size+fl_size+br_size)); }
		}
	}
	conti_radar::Measurement f_out, fr_out, fl_out, br_out, bl_out;
	f_out = contiMsgOutlierFilterComp(f_input, f_outliers, beta_f, calcVjxVjy(ego_mat, f_x, f_y));
	fr_out = contiMsgOutlierFilterComp(fr_input, fr_outliers, beta_fr, calcVjxVjy(ego_mat, fr_x, fr_y));
	fl_out = contiMsgOutlierFilterComp(fl_input, fl_outliers, beta_fl, calcVjxVjy(ego_mat, fl_x, fl_y));
	br_out = contiMsgOutlierFilterComp(br_input, br_outliers, beta_br, calcVjxVjy(ego_mat, br_x, br_y));
	bl_out = contiMsgOutlierFilterComp(bl_input, bl_outliers, beta_bl, calcVjxVjy(ego_mat, bl_x, bl_y));
	//-- Outliers Msg End --//

	conti_radar::Measurement f_in, fr_in, fl_in, br_in, bl_in;
	f_in = contiMsgInlierFilter(f_input, outlierToInlier(f_outliers,f_size));
	fr_in = contiMsgInlierFilter(fr_input, outlierToInlier(fr_outliers,fr_size));
	fl_in = contiMsgInlierFilter(fl_input, outlierToInlier(fl_outliers,fl_size));
	br_in = contiMsgInlierFilter(br_input, outlierToInlier(br_outliers,br_size));
	bl_in = contiMsgInlierFilter(bl_input, outlierToInlier(bl_outliers,bl_size));


	f_outlier_pub.publish(f_out);
	fr_outlier_pub.publish(fr_out);
	fl_outlier_pub.publish(fl_out);
	br_outlier_pub.publish(br_out);
	bl_outlier_pub.publish(bl_out);

	f_inlier_pub.publish(f_in);
	fr_inlier_pub.publish(fr_in);
	fl_inlier_pub.publish(fl_in);
	br_inlier_pub.publish(br_in);
	bl_inlier_pub.publish(bl_in);

  pub.publish(odom);
}


int main (int argc, char** argv)
{

  ros::init (argc, argv, "radar_ego_motion_valid");
  ros::NodeHandle nh;

  message_filters::Subscriber<conti_radar::Measurement> sub_f(nh, "/radar_front", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_fl(nh, "/radar_front_left", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_fr(nh, "/radar_front_right", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_bl(nh, "/radar_back_left", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_br(nh, "/radar_back_right", 1);

  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(3), sub_f, sub_fl, sub_fr, sub_bl, sub_br);
  no_cloud_sync_->registerCallback(boost::bind(&Callback, _1, _2, _3, _4, _5));

  pub = nh.advertise<nav_msgs::Odometry> ("vel_valid", 1);

  f_outlier_pub  = nh.advertise<conti_radar::Measurement> ("radar_front_outlier_valid", 1);
  fr_outlier_pub = nh.advertise<conti_radar::Measurement> ("radar_front_right_outlier_valid", 1);
  fl_outlier_pub = nh.advertise<conti_radar::Measurement> ("radar_front_left_outlier_valid", 1);
  br_outlier_pub = nh.advertise<conti_radar::Measurement> ("radar_back_right_outlier_valid", 1);
  bl_outlier_pub = nh.advertise<conti_radar::Measurement> ("radar_back_left_outlier_valid", 1);

  f_inlier_pub  = nh.advertise<conti_radar::Measurement> ("radar_front_inlier_valid", 1);
  fr_inlier_pub = nh.advertise<conti_radar::Measurement> ("radar_front_right_inlier_valid", 1);
  fl_inlier_pub = nh.advertise<conti_radar::Measurement> ("radar_front_left_inlier_valid", 1);
  br_inlier_pub = nh.advertise<conti_radar::Measurement> ("radar_back_right_inlier_valid", 1);
  bl_inlier_pub = nh.advertise<conti_radar::Measurement> ("radar_back_left_inlier_valid", 1);

  ros::spin ();
}

