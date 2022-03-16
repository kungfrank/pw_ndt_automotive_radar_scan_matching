#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigenvalues>

#include <sensor_msgs/PointCloud2.h>
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

#include <math.h>
#include <complex>
#include <iostream>
using namespace std;
using namespace cv;

void cb(const sensor_msgs::PointCloud2ConstPtr& input);
vector< vector<float> > Descriptor(vector<Point2f> data);
vector< vector<int> > UnaryMatches(vector< vector<float> > pre_descriptor \
								 									,vector< vector<float> > curr_descriptor);
Eigen::MatrixXf PairwiseCompatibilityScores(vector< vector<int> > U \
																						,vector<cv::Point2f> pre_data \
																						,vector<cv::Point2f> curr_data);
float UpdateScore(Eigen::MatrixXf C, Eigen::MatrixXf m_);
Eigen::Matrix4f getTransform(float x,float y, float theta);
Eigen::MatrixXf VectorToEigenMatrix(vector< vector<float> > vectorf2d);

vector< vector<float> > pre_descriptor(0, vector<float> (0, 0));
vector<cv::Point2f> pre_data;
bool init_flag = false;

//////////////////////////-- main callback --//////////////////////////

void cb(const sensor_msgs::PointCloud2ConstPtr& input){

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

  cv::Mat image(3000, 3000, CV_8UC3, Scalar(255,255,255));

	//-- access point --//
	vector<cv::Point2f> curr_data;
	//cout << "Original PC Size: " << in_pc->size() << endl;
	///draw and assign curr_data///
	for(int i=0 ; i<in_pc->size(); i++){
		float dis = sqrt(pow(in_pc->points[i].x,2)+pow(in_pc->points[i].y,2));
		if(dis < 150){ // remove points far than 150m from center //
			cv::Point2f p(300-(in_pc->points[i].y + 150), 300-(in_pc->points[i].x + 150));
			cv::circle(image, cv::Point2f(p.x*10,p.y*10), 4, Scalar(0, 200, 0),8);
			curr_data.push_back(p);
		}
	}
	cout << "Data Size: " << curr_data.size() << endl;
	///draw pre_data///
	for(int i=0 ; i<pre_data.size(); i++){
		cv::circle(image, cv::Point2f(pre_data[i].x*10,pre_data[i].y*10), 4, Scalar(0, 0, 255),8);
	}
	//-- Generate Descriptor --//
	vector< vector<float> > curr_descriptor(0, vector<float> (0, 0));
	curr_descriptor = Descriptor(curr_data);

	vector< vector<int> > U; // N x 2
	Eigen::MatrixXf C_(U.size(), U.size()); // N x N
	Eigen::MatrixXf V_star(U.size(), 1); // N x 1
	Eigen::MatrixXf m_;

	if(init_flag){
		//-- UnaryMatches --//
		U = UnaryMatches(pre_descriptor, curr_descriptor);
		//-- Pairwise Compatibility Scores --//
		C_ = PairwiseCompatibilityScores(U, pre_data, curr_data);
		Eigen::EigenSolver<Eigen::MatrixXf> s(C_);
		std::cout << "eigenvalue:" << std::endl;
		std::cout << s.eigenvalues().transpose() << std::endl;
		std::cout << "max eigenvalue:" << std::endl;
		Eigen::MatrixXf::Index maxIndex;
		std::cout << s.eigenvalues().real().maxCoeff(&maxIndex) << std::endl; // TODO current version is incorrect, it should be re^2+im^2 //
		std::cout << "first eigenvector:" << std::endl;
		std::cout << s.eigenvectors().col(0).transpose() << std::endl;
		V_star = s.eigenvectors().col(maxIndex).real();
		std::cout << "principal eigenvector=" << std::endl;
		std::cout << V_star.transpose() << std::endl;
		std::cout << "principal eigenvector abs2=" << std::endl;
		std::cout << V_star.cwiseAbs2().transpose() << std::endl;
		//std::vector<int> M(U.size());
		//std::iota (std::begin(M), std::end(M), 0); // assign value from 0 to U.size() into vector M

		int count = 0;
		float score = 0;
		float old_score = 0;
		m_ = Eigen::MatrixXf::Constant(U.size(), 1, 0.0); // N x 1

		while(count < U.size()){
			int i_row, i_col;
			cout << "Max value in V_star abs2: " << V_star.cwiseAbs2().maxCoeff(&i_row, &i_col);
			cout << " index: " << i_row << endl;
			V_star(i_row, i_col) = 0.0f;
			m_(i_row,0) = 1.0f;
			//cout << "m:" << m.transpose() << endl;
			score = UpdateScore(C_,m_); //.cast<float>()
			cout << "count: " << count << endl;
			cout << "score: " << score << endl;
			if(score < old_score){break;}
			count++;
			old_score = score;
		}
		cout << "Done !!!" << endl;

	}
	else{init_flag = true;}

	///draw association//
	for(int i=0 ; i<U.size(); i++){
		if(m_(i,0) == 1){
			int pre_index, curr_index;
			pre_index = U[i][0];
			curr_index = U[i][1];
			cv::Point2f p1(pre_data[pre_index].x*10,pre_data[pre_index].y*10);
			cv::Point2f p2(curr_data[curr_index].x*10,curr_data[curr_index].y*10);
			cv::line(image, p1, p2, Scalar(255, 0, 0), 1, CV_AA);
		}
		else{
			int pre_index, curr_index;
			pre_index = U[i][0];
			curr_index = U[i][1];
			cv::Point2f p1(pre_data[pre_index].x*10,pre_data[pre_index].y*10);
			cv::Point2f p2(curr_data[curr_index].x*10,curr_data[curr_index].y*10);
			cv::line(image, p1, p2, Scalar(0, 150, 150), 1, CV_AA);
		}
	}

	pre_descriptor = curr_descriptor; // store old //
	pre_data = curr_data; // store old //

  namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.
  imshow( "Display window", image );             // Show our image inside it.
  waitKey(0);                                    // Wait for a keystroke in the window

	//ROS_INFO("still working");

	curr_data.clear();
	curr_descriptor.clear();
	U.clear();
}

//////////////////////////-- main callback end --//////////////////////////

float UpdateScore(Eigen::MatrixXf C_, Eigen::MatrixXf m_){
	Eigen::MatrixXf t1 = (m_.transpose()*C_*m_);
	Eigen::MatrixXf t2 = (m_.transpose()*m_);
	float score = t1(0,0)/t2(0,0);
	return score;
}

Eigen::MatrixXf PairwiseCompatibilityScores(vector< vector<int> > U \
																						,vector<cv::Point2f> pre_data \
																						,vector<cv::Point2f> curr_data){
	Eigen::MatrixXf C(U.size(), U.size()); // N x N

	for(int i=0; i<U.size(); i++){
		int i_pre, i_curr;
		i_pre = U[i][0];
		i_curr = U[i][1];
		for(int j=0; j<=i; j++){
			if(i==j){C(i,j) = U.size();}
			else{
				int j_pre, j_curr;
				j_pre = U[j][0];
				j_curr = U[j][1];
				float d_pre, d_curr;
				d_pre = pow(pre_data[i_pre].x-pre_data[j_pre].x,2)+pow(pre_data[i_pre].y-pre_data[j_pre].y,2);
				d_curr = pow(curr_data[i_curr].x-curr_data[j_curr].x,2)+pow(curr_data[i_curr].y-curr_data[j_curr].y,2);
				C(i,j) = 1.0/(1.0+abs(d_pre-d_curr));
				C(j,i) = 1.0/(1.0+abs(d_pre-d_curr));
			}
		}
	}
	return C;
}

vector< vector<int> > UnaryMatches(vector< vector<float> > pre_descriptor \
								 									,vector< vector<float> > curr_descriptor){
	vector< vector<int> > U; //U(pre_descriptor.size(), vector<int> (2, 0));
	//cout << "pre_descriptor.size(): " << pre_descriptor.size() << endl;
	//cout << "curr_descriptor.size(): " << curr_descriptor.size() << endl;
	Eigen::MatrixXf pre_des, curr_des;

	pre_des = VectorToEigenMatrix(pre_descriptor);
	curr_des = VectorToEigenMatrix(curr_descriptor);

	//cout << "pre row&col: " << pre_des.rows() << " " << pre_des.cols() << endl;
	//cout << "curr row&col: " << curr_des.rows() << " " << curr_des.cols() << endl;

	Eigen::MatrixXf pre_i(curr_des.rows(),curr_des.cols());
	Eigen::MatrixXf a(pre_des.rows(),1);
	a = Eigen::MatrixXf::Ones(curr_des.rows(),1);

	for(int i=0;i<pre_des.rows();i++){
		//for(int j=0;j<curr_des.rows();j++){pre_i.row(j) = pre_des.row(i);}
		//ROS_INFO("Start"); // 0.02s
		pre_i = a*pre_des.row(i);

		float min_value; int min_index;

		min_value = (pre_i - curr_des).cwiseAbs().rowwise().sum().minCoeff(&min_index);
		//cout <<"i: "<<i<<" min value: "<<min_value<<" index: "<<min_index<<endl;
		vector<int> pair;
		pair.push_back(i); pair.push_back(min_index);
		U.push_back(pair);
		//cout << "U[0].size(): " << U[0].size() << endl;
		//cout << "U.size(): " << U.size() << endl;
	}

	return U;
}

Eigen::MatrixXf VectorToEigenMatrix(vector< vector<float> > vectorf2d){
	Eigen::MatrixXf mat(vectorf2d.size(),vectorf2d[0].size());
	for(int i=0;i<vectorf2d.size();i++){
		for(int j=0;j<vectorf2d[0].size();j++){
			mat(i,j) = vectorf2d[i][j];
		}
	}
	return mat;
}

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

vector< vector<float> > Descriptor(vector<Point2f> data){
	//descriptor resolution: 4.5 degree 40 cm
	vector< vector<float> > descriptor_vec;
	vector< vector<float> > descriptor_ang_vec;
	vector< vector<float> > descriptor_dis_vec;

	for(int i=0 ; i<data.size(); i++){
		float ori_x = data[i].x;
		float ori_y = data[i].y;

		vector<float> descriptor_ang(80,0.0);
		vector<float> descriptor_dis(750,0.0);

		for(int j=0 ; j<data.size(); j++){
			if(i!=j){
				data[j].x = data[j].x - ori_x;
				data[j].y = data[j].y - ori_y;

				int index;
				/// slice descriptor ///
				float theta;
				theta = atan2(data[j].x, data[j].y);
				theta = theta / M_PI *180;
				if(theta<0){theta = theta+360;}
				index = theta / 4.5f;
				if(index == 80){descriptor_ang[0] = descriptor_ang[0]+1.0f;}
				else{descriptor_ang[index] = descriptor_ang[index]+1.0f;}

				/// annuli descriptor ///
				float dis;
				dis = sqrt(pow(data[j].x,2)+pow(data[j].y,2));
				index = dis / 0.4f;
				//cout << "dis & index: "<< dis << " " <<index << endl;
				if(index >= 750){descriptor_dis[749] = descriptor_dis[749]+1.0f;}
				else{descriptor_dis[index] = descriptor_dis[index]+1.0f;}
			}
		}
		//-- Resort slice descriptor --//
		int i_max = distance(descriptor_ang.begin(), max_element(descriptor_ang.begin(), descriptor_ang.end()));
		for(int i=0;i<i_max;i++){
			float temp = descriptor_ang[i];
			descriptor_ang.erase(descriptor_ang.begin());
			descriptor_ang.push_back(temp);
		}
		//-- Normalize --//
		for(int k=0 ; k<80; k++){
			descriptor_ang[k] = descriptor_ang[k]/float(data.size()-1)*10.0f;
		}
		for(int k=0 ; k<750; k++){
			descriptor_dis[k] = descriptor_dis[k]/float(data.size()-1)*10.0f;
		}
		//-- Store --//
		descriptor_ang_vec.push_back(descriptor_ang);
		descriptor_dis_vec.push_back(descriptor_dis);
		descriptor_ang.clear();
		descriptor_dis.clear();
	}
	//-- Combine --//
	for(int i=0; i<data.size(); i++){
		vector<float> v;
		v.insert(v.begin(),descriptor_ang_vec[i].begin(),descriptor_ang_vec[i].end());
		v.insert(v.end(),descriptor_dis_vec[i].begin(),descriptor_dis_vec[i].end());
		descriptor_vec.push_back(v);
	}
	/*
	for(int k=0; k<750; k++){
		cout << "point:0 annuli:"<< k << " value=" <<descriptor_dis_vec[0][k] << endl; 
	}*/
	return descriptor_vec;
	descriptor_ang_vec.clear();
	descriptor_dis_vec.clear();
	descriptor_vec.clear();
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "data_association");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/output", 1, cb);

  ros::spin ();
}

