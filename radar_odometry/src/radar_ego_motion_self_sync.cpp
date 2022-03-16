/*
This is the modify radar_ego_motion code, replace msg filter with self sync code. Sync all first arrived msg.
*/

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

conti_radar::Measurement::Ptr f_input;
conti_radar::Measurement::Ptr fr_input;
conti_radar::Measurement::Ptr fl_input;
conti_radar::Measurement::Ptr br_input;
conti_radar::Measurement::Ptr bl_input;

bool f_flag = false;
bool fr_flag = false;
bool fl_flag = false;
bool br_flag = false;
bool bl_flag = false;

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
	//for(int i=0;i<(input->points.size());i++)if ((input->points[i].invalid_state) == 0x00){valid_size++;}
	valid_size = input->points.size();
	cout << "size: " << valid_size << endl;

	VD_mat.resize(valid_size,1); // N x 1
	M_mat.resize(valid_size, 2); // N x 2
	R_mat.resize(valid_size, 2); // N x 2

	int n=0;
	for(int i=0; i< (input->points.size()); i++){
		//if ((input->points[i].invalid_state) == 0x00){
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
		//}
	}

	Eigen::MatrixXd S_mat;
	//-- for general condition (omega,vx,vy)--//
/*
	S_mat.resize(2, 3); // 2 x 3
	S_mat(0,0) = -r_y; S_mat(0,1) = 1.0; S_mat(0,2) = 0.0;
	S_mat(1,0) = r_x;  S_mat(1,1) = 0.0; S_mat(1,2) = 1.0;
	R_mat = M_mat*S_mat; // N x 3
*/
	//-- for ackerman condition (omega,vx)--//
	S_mat.resize(2, 2); // 2 x 2
	S_mat(0,0) = -r_y; S_mat(0,1) = 1.0;
	S_mat(1,0) = r_x;  S_mat(1,1) = 0.0;
	R_mat = M_mat*S_mat; // N x 2

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
	radar_v_from_center.setOrigin( tf::Vector3(vj(0,0), vj(1,0), 0.0) );

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

vector<int> outlierToInlier(vector<int> outlier, int size) {
	vector<int> inlier;
	if(outlier.size()>0){
		int j=0;
		for(int i=0;i<size;i++){
			if(i == outlier[j]){ j++;}
			else{ inlier.push_back(i);}
		}
	}
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
	S_mat.resize(2, 2); // 2 x 2
	S_mat(0,0) = -r_y; S_mat(0,1) = 1.0;
	S_mat(1,0) = r_x;  S_mat(1,1) = 0.0;
	Vj = S_mat * ego_mat;
	return Vj;
}


void FullCallback (conti_radar::Measurement::Ptr& f_input,
              		 conti_radar::Measurement::Ptr& fl_input,
           		     conti_radar::Measurement::Ptr& fr_input,
           		     conti_radar::Measurement::Ptr& bl_input,
           		     conti_radar::Measurement::Ptr& br_input) 
{
	double f_t = f_input->header.stamp.toSec();
	double fl_t = fl_input->header.stamp.toSec();
	double fr_t = fr_input->header.stamp.toSec();
	double bl_t = bl_input->header.stamp.toSec();
	double br_t = br_input->header.stamp.toSec();
	double list[] = {f_t,fl_t,fr_t,bl_t,br_t};

	std::cout << std::fixed; std::cout.precision(5);

  cout << "front time: " << f_t << endl;
  cout << "fl time: " << fl_t << endl;
  cout << "fr time: " << fr_t << endl;
  cout << "bl time: " << bl_t << endl;
  cout << "br time: " << br_t << endl;

	double min_t = *std::min_element(list,list+5);
  cout << "biggest time difference: " << (*std::max_element(list,list+5)-*std::min_element(list,list+5)) << endl;
	cout << endl;

	Eigen::MatrixXd M_mat_f;
	Eigen::MatrixXd R_mat_f;
	Eigen::MatrixXd VD_mat_f;
	float temp_bias = 0; //1.5
	float f_x = 3.41199994087 + temp_bias; //3.6
	float f_y = 0.0 + temp_bias;
	float beta_f = 0.003;
	radar_sensor(f_input, beta_f, f_x, f_y, M_mat_f, R_mat_f, VD_mat_f);

	Eigen::MatrixXd M_mat_fr;
	Eigen::MatrixXd R_mat_fr;
	Eigen::MatrixXd VD_mat_fr;
	float fr_x = 2.42199993134 + temp_bias;
	float fr_y = -0.800000011921 - temp_bias;
	float beta_fr = -1.588;
	radar_sensor(fr_input, beta_fr, fr_x, fr_y, M_mat_fr, R_mat_fr, VD_mat_fr);

	Eigen::MatrixXd M_mat_fl;
	Eigen::MatrixXd R_mat_fl;
	Eigen::MatrixXd VD_mat_fl;
	float fl_x = 2.42199993134 + temp_bias;
	float fl_y = 0.800000011921 + temp_bias;
	float beta_fl = 1.542;
	radar_sensor(fl_input, beta_fl, fl_x, fl_y, M_mat_fl, R_mat_fl, VD_mat_fl);

	Eigen::MatrixXd M_mat_br;
	Eigen::MatrixXd R_mat_br;
	Eigen::MatrixXd VD_mat_br;
	float br_x = -0.561999976635 - temp_bias;
	float br_y = -0.617999970913 - temp_bias;
	float beta_br = -3.074;
	radar_sensor(br_input, beta_br, br_x, br_y, M_mat_br, R_mat_br, VD_mat_br);

	Eigen::MatrixXd M_mat_bl;
	Eigen::MatrixXd R_mat_bl;
	Eigen::MatrixXd VD_mat_bl;
	float bl_x = -0.561999976635 - temp_bias;
	float bl_y = 0.628000020981 + temp_bias;
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
	inliers = calcRansacMat(VD, R, 0.12);
  Eigen::MatrixXd RANSAC_R = deleteCols(R, inliers);
  Eigen::MatrixXd RANSAC_VD = deleteCols(VD, inliers);
	cout << "R.rows(): " << R.rows() << endl;
	cout << "RANSAC_R.rows(): " << RANSAC_R.rows() << endl;
	//-- End RANSAC --//

	//-- Vel Msg--//
	Eigen::MatrixXd ego_mat = RANSAC_R.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(RANSAC_VD);

	cout << "w: " << ego_mat(0,0)/M_PI*180 << endl;
	cout << "vx: " << ego_mat(1,0) << endl;
	// cout << "vy: " << ego_mat(2,0) << endl;
	cout << endl;

	nav_msgs::Odometry odom;
  odom.header.frame_id = "/nuscenes_radar_front"; // TODO should be car center
  odom.header.stamp = ros::Time(min_t);
  odom.twist.twist.linear.x = ego_mat(1,0);
  odom.twist.twist.angular.z = ego_mat(0,0)/M_PI*180;
	//-- Vel Msg End --//

	//-- Radar Vel Msg --//
	Eigen::MatrixXd vj_f = calcVjxVjy(ego_mat, f_x, f_y);
	Eigen::MatrixXd vj_fr = calcVjxVjy(ego_mat, fr_x, fr_y);
	Eigen::MatrixXd vj_fl = calcVjxVjy(ego_mat, fl_x, fl_y);
	Eigen::MatrixXd vj_br = calcVjxVjy(ego_mat, br_x, br_y);
	Eigen::MatrixXd vj_bl = calcVjxVjy(ego_mat, bl_x, bl_y);
	//cout << "front vel: " << vj_f(0,0) << ", " << vj_f(1,0) << endl;
	//cout << "front r vel: " << vj_fr(0,0) << ", " << vj_fr(1,0) << endl;
	//cout << "front l vel: " << vj_fl(0,0) << ", " << vj_fl(1,0) << endl;
	//cout << "back r vel: " << vj_br(0,0) << ", " << vj_br(1,0) << endl;
	//cout << "back l vel: " << vj_bl(0,0) << ", " << vj_bl(1,0) << endl;

	nav_msgs::Odometry f_v_c;
  f_v_c.header.stamp = ros::Time(min_t);
  f_v_c.twist.twist.linear.x = vj_f(0,0);  f_v_c.twist.twist.linear.y = vj_f(1,0);
	nav_msgs::Odometry fr_v_c;
  fr_v_c.header.stamp = ros::Time(min_t);
  fr_v_c.twist.twist.linear.x = vj_fr(0,0);  fr_v_c.twist.twist.linear.y = vj_fr(1,0);
	nav_msgs::Odometry fl_v_c;
  fl_v_c.header.stamp = ros::Time(min_t);
  fl_v_c.twist.twist.linear.x = vj_fl(0,0);  fl_v_c.twist.twist.linear.y = vj_fl(1,0);
	nav_msgs::Odometry br_v_c;
  br_v_c.header.stamp = ros::Time(min_t);
  br_v_c.twist.twist.linear.x = vj_br(0,0);  br_v_c.twist.twist.linear.y = vj_br(1,0);
	nav_msgs::Odometry bl_v_c;
  bl_v_c.header.stamp = ros::Time(min_t);
  bl_v_c.twist.twist.linear.x = vj_bl(0,0);  bl_v_c.twist.twist.linear.y = vj_bl(1,0);
	//-- Radar Vel Msg End --//

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
	f_out = contiMsgOutlierFilterComp(f_input, f_outliers, beta_f, vj_f);
	fr_out = contiMsgOutlierFilterComp(fr_input, fr_outliers, beta_fr, vj_fr);
	fl_out = contiMsgOutlierFilterComp(fl_input, fl_outliers, beta_fl, vj_fl);
	br_out = contiMsgOutlierFilterComp(br_input, br_outliers, beta_br, vj_br);
	bl_out = contiMsgOutlierFilterComp(bl_input, bl_outliers, beta_bl, vj_bl);
	//-- Outliers Msg End --//

////////////////////////////////////////////////////////////////////////////////////
	std::vector<int> f_inliers, fr_inliers, fl_inliers, br_inliers, bl_inliers;
	f_inliers = outlierToInlier(f_outliers,f_size);
	fr_inliers = outlierToInlier(fr_outliers,fr_size);
	fl_inliers = outlierToInlier(fl_outliers,fl_size);
	br_inliers = outlierToInlier(br_outliers,br_size);
	bl_inliers = outlierToInlier(bl_outliers,bl_size);

	Eigen::MatrixXd f_v_mat(2,1);
	f_v_mat = calcVj(deleteCols(M_mat_f, f_inliers), deleteCols(VD_mat_f, f_inliers));
	nav_msgs::Odometry f_v;
  f_v.header.stamp = ros::Time(min_t);
  f_v.twist.twist.linear.x = f_v_mat(0,0);  f_v.twist.twist.linear.y = f_v_mat(1,0);
	//cout << "f_v: " << f_v_mat(0,0) << ", " << f_v_mat(1,0) << endl;

	Eigen::MatrixXd fr_v_mat(2,1);
	fr_v_mat = calcVj(deleteCols(M_mat_fr, fr_inliers), deleteCols(VD_mat_fr, fr_inliers));
	nav_msgs::Odometry fr_v;
  fr_v.header.stamp = ros::Time(min_t);
  fr_v.twist.twist.linear.x = fr_v_mat(0,0);  fr_v.twist.twist.linear.y = fr_v_mat(1,0);

	Eigen::MatrixXd fl_v_mat(2,1);
	fl_v_mat = calcVj(deleteCols(M_mat_fl, fl_inliers), deleteCols(VD_mat_fl, fl_inliers));
	nav_msgs::Odometry fl_v;
  fl_v.header.stamp = ros::Time(min_t);
  fl_v.twist.twist.linear.x = fl_v_mat(0,0);  fl_v.twist.twist.linear.y = fl_v_mat(1,0);

	Eigen::MatrixXd br_v_mat(2,1);
	br_v_mat = calcVj(deleteCols(M_mat_br, br_inliers), deleteCols(VD_mat_br, br_inliers));
	nav_msgs::Odometry br_v;
  br_v.header.stamp = ros::Time(min_t);
  br_v.twist.twist.linear.x = br_v_mat(0,0);  br_v.twist.twist.linear.y = br_v_mat(1,0);

	Eigen::MatrixXd bl_v_mat(2,1);
	bl_v_mat = calcVj(deleteCols(M_mat_bl, bl_inliers), deleteCols(VD_mat_bl, bl_inliers));
	nav_msgs::Odometry bl_v;
  bl_v.header.stamp = ros::Time(min_t);
  bl_v.twist.twist.linear.x = bl_v_mat(0,0);  bl_v.twist.twist.linear.y = bl_v_mat(1,0);

//////////////////////////////////////////////////////////////////////////////////////

	f_outlier_pub.publish(f_out);
	fr_outlier_pub.publish(fr_out);
	fl_outlier_pub.publish(fl_out);
	br_outlier_pub.publish(br_out);
	bl_outlier_pub.publish(bl_out);

  f_vel_pub.publish(f_v);
  fr_vel_pub.publish(fr_v);
  fl_vel_pub.publish(fl_v);
  br_vel_pub.publish(br_v);
  bl_vel_pub.publish(bl_v);

  f_vel_c_pub.publish(f_v_c);
  fr_vel_c_pub.publish(fr_v_c);
  fl_vel_c_pub.publish(fl_v_c);
  br_vel_c_pub.publish(br_v_c);
  bl_vel_c_pub.publish(bl_v_c);

  pub.publish(odom);
}

void checkFlag(bool f_flag, bool fr_flag, bool fl_flag, bool br_flag, bool bl_flag){
	if(f_flag == true && fr_flag == true && fl_flag == true && br_flag == true && bl_flag == true){
		FullCallback(f_input, fr_input, fl_input, br_input, bl_input);
		f_flag = false;
		fr_flag = false;
		fl_flag = false;
		br_flag = false;
		bl_flag = false;
	}
}

void f_cb(conti_radar::Measurement::Ptr input){
	f_input = input;
	f_flag = true;
	checkFlag(f_flag, fr_flag, fl_flag, br_flag, bl_flag);
}

void fr_cb(conti_radar::Measurement::Ptr input){
	fr_input = input;
	fr_flag = true;
	checkFlag(f_flag, fr_flag, fl_flag, br_flag, bl_flag);
}
void fl_cb(conti_radar::Measurement::Ptr input){
	fl_input = input;
	fl_flag = true;
	checkFlag(f_flag, fr_flag, fl_flag, br_flag, bl_flag);
}
void br_cb(conti_radar::Measurement::Ptr input){
	br_input = input;
	br_flag = true;
	checkFlag(f_flag, fr_flag, fl_flag, br_flag, bl_flag);
}
void bl_cb(conti_radar::Measurement::Ptr input){ // const conti_radar::MeasurementConstPtr&
	bl_input = input;
	bl_flag = true;
	checkFlag(f_flag, fr_flag, fl_flag, br_flag, bl_flag);
}


int main (int argc, char** argv)
{

  ros::init (argc, argv, "radar_ego_motion_2hz");
  ros::NodeHandle nh;

  ros::Subscriber sub_f = nh.subscribe ("/radar_front", 1, f_cb);
  ros::Subscriber sub_fl = nh.subscribe ("/radar_front_left", 1, fl_cb);
  ros::Subscriber sub_fr = nh.subscribe ("/radar_front_right", 1, fr_cb);
  ros::Subscriber sub_bl = nh.subscribe ("/radar_back_left", 1, bl_cb);
  ros::Subscriber sub_br = nh.subscribe ("/radar_back_right", 1, br_cb);

  pub = nh.advertise<nav_msgs::Odometry> ("vel", 1);

  f_outlier_pub = nh.advertise<conti_radar::Measurement> ("radar_front_outlier", 1);
  fr_outlier_pub = nh.advertise<conti_radar::Measurement> ("radar_front_right_outlier", 1);
  fl_outlier_pub = nh.advertise<conti_radar::Measurement> ("radar_front_left_outlier", 1);
  br_outlier_pub = nh.advertise<conti_radar::Measurement> ("radar_back_right_outlier", 1);
  bl_outlier_pub = nh.advertise<conti_radar::Measurement> ("radar_back_left_outlier", 1);

  f_vel_pub = nh.advertise<nav_msgs::Odometry> ("f_vel", 1);
  fr_vel_pub = nh.advertise<nav_msgs::Odometry> ("fr_vel", 1);
  fl_vel_pub = nh.advertise<nav_msgs::Odometry> ("fl_vel", 1);
  br_vel_pub = nh.advertise<nav_msgs::Odometry> ("br_vel", 1);
  bl_vel_pub = nh.advertise<nav_msgs::Odometry> ("bl_vel", 1);

  f_vel_c_pub = nh.advertise<nav_msgs::Odometry> ("f_vel_from_c", 1);
  fr_vel_c_pub = nh.advertise<nav_msgs::Odometry> ("fr_vel_from_c", 1);
  fl_vel_c_pub = nh.advertise<nav_msgs::Odometry> ("fl_vel_from_c", 1);
  br_vel_c_pub = nh.advertise<nav_msgs::Odometry> ("br_vel_from_c", 1);
  bl_vel_c_pub = nh.advertise<nav_msgs::Odometry> ("bl_vel_from_c", 1);

  ros::spin ();
}

