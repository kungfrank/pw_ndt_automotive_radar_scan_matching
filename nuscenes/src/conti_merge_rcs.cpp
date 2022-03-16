#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <conti_radar/Measurement.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <string>
using namespace std;

ros::Publisher valid_pub;
ros::Publisher invalid_pub;
ros::Publisher pub_07;
ros::Publisher pub_04;
sensor_msgs::PointCloud2 output;

typedef message_filters::sync_policies::ApproximateTime<conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        conti_radar::Measurement,
                                                        conti_radar::Measurement,
																												conti_radar::Measurement> NoCloudSyncPolicy;

////////////////////////////////// Variable for Type Counting //////////////////////////////

int invalid_1, invalid_2, invalid_3, invalid_5, invalid_6, invalid_7, invalid_d, invalid_e;
int valid_0, valid_4, valid_8, valid_9, valid_a, valid_b, valid_c, valid_f, valid_10, valid_11;

////////////////////////////////// Variable for Type Counting End //////////////////////////

void PrintTypeCount(){
	cout << "====================== " << endl;
	cout << "invalid_1: " << invalid_1 << endl;
	cout << "invalid_2: " << invalid_2 << endl;
	cout << "invalid_3: " << invalid_3 << endl;
	cout << "invalid_5: " << invalid_5 << endl;
	cout << "invalid_6: " << invalid_6 << endl;
	cout << "invalid_7: " << invalid_7 << endl;
	cout << "invalid_d: " << invalid_d << endl;
	cout << "invalid_e: " << invalid_e << endl;
	cout << "---------------------- " << endl;
	cout << "valid_0: " << valid_0 << endl;
	cout << "valid_4: " << valid_4 << endl;
	cout << "valid_8: " << valid_8 << endl;
	cout << "valid_9: " << valid_9 << endl;
	cout << "valid_a: " << valid_a << endl;
	cout << "valid_b: " << valid_b << endl;
	cout << "valid_c: " << valid_c << endl;
	cout << "valid_f: " << valid_f << endl;
	cout << "valid_10: " << valid_10 << endl;
	cout << "valid_11: " << valid_11 << endl;
	cout << "====================== " << endl;
}

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

void conti_filter(const conti_radar::MeasurementConstPtr& conti_input, sensor_msgs::PointCloud2::Ptr ros_pc2, sensor_msgs::PointCloud2::Ptr ros_pc2_i, sensor_msgs::PointCloud2::Ptr ros_pc2_07, sensor_msgs::PointCloud2::Ptr ros_pc2_04){

	pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PCLPointCloud2 pc2;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_i(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PCLPointCloud2 pc2_i;

	//
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_07(new pcl::PointCloud<pcl::PointXYZI>); // Invalid cluster because outside sensor field of view
	pcl::PCLPointCloud2 pc2_07;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_04(new pcl::PointCloud<pcl::PointXYZI>); // Valid cluster with low RCS
	pcl::PCLPointCloud2 pc2_04;
	//

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
		else if ((conti_input->points[i].invalid_state) == 0x07){
			point.x = conti_input->points[i].longitude_dist;
			point.y = conti_input->points[i].lateral_dist;
			point.z = 0;
			point.intensity = conti_input->points[i].rcs;
			pc_07->points.push_back(point);
		}
		else if ((conti_input->points[i].invalid_state) == 0x0b){
			point.x = conti_input->points[i].longitude_dist;
			point.y = conti_input->points[i].lateral_dist;
			point.z = 0;
			point.intensity = conti_input->points[i].rcs;
			pc_04->points.push_back(point);
		}
		else{
			point.x = conti_input->points[i].longitude_dist;
			point.y = conti_input->points[i].lateral_dist;
			point.z = 0;
			point.intensity = conti_input->points[i].rcs;
			pc_i->points.push_back(point);
		}

	////////////////////////////////// Count Type //////////////////////////////
		if ((conti_input->points[i].invalid_state) == 0x01){invalid_1++;}
		else if ((conti_input->points[i].invalid_state) == 0x02){invalid_2++;}
		else if ((conti_input->points[i].invalid_state) == 0x03){invalid_5++;}
		else if ((conti_input->points[i].invalid_state) == 0x05){invalid_5++;}
		else if ((conti_input->points[i].invalid_state) == 0x06){invalid_6++;}
		else if ((conti_input->points[i].invalid_state) == 0x07){invalid_7++;}
		else if ((conti_input->points[i].invalid_state) == 0x0d){invalid_d++;}
		else if ((conti_input->points[i].invalid_state) == 0x0e){invalid_e++;}

		else if ((conti_input->points[i].invalid_state) == 0x00){valid_0++;}
		else if ((conti_input->points[i].invalid_state) == 0x04){valid_4++;}
		else if ((conti_input->points[i].invalid_state) == 0x08){valid_8++;}
		else if ((conti_input->points[i].invalid_state) == 0x09){valid_9++;}
		else if ((conti_input->points[i].invalid_state) == 0x0a){valid_a++;}
		else if ((conti_input->points[i].invalid_state) == 0x0b){valid_b++;}
		else if ((conti_input->points[i].invalid_state) == 0x0c){valid_c++;}
		else if ((conti_input->points[i].invalid_state) == 0x0f){valid_f++;}
		else if ((conti_input->points[i].invalid_state) == 0x10){valid_10++;}
		else if ((conti_input->points[i].invalid_state) == 0x11){valid_11++;}
	////////////////////////////////// End Count Type //////////////////////////////

	}
	//cout << "valid num: " << pc->points.size() << endl;
	pcl::toPCLPointCloud2(*pc, pc2);
  pcl_conversions::fromPCL(pc2, *ros_pc2);
	ros_pc2->is_dense = true;

	pcl::toPCLPointCloud2(*pc_i, pc2_i);
  pcl_conversions::fromPCL(pc2_i, *ros_pc2_i);
	ros_pc2_i->is_dense = true;

	pcl::toPCLPointCloud2(*pc_07, pc2_07);
  pcl_conversions::fromPCL(pc2_07, *ros_pc2_07);
	ros_pc2_07->is_dense = true;
	pcl::toPCLPointCloud2(*pc_04, pc2_04);
  pcl_conversions::fromPCL(pc2_04, *ros_pc2_04);
	ros_pc2_04->is_dense = true;
}

sensor_msgs::PointCloud2 merge( const sensor_msgs::PointCloud2::Ptr& f,
 																const sensor_msgs::PointCloud2::Ptr& fl,
 																const sensor_msgs::PointCloud2::Ptr& fr,
 																const sensor_msgs::PointCloud2::Ptr& bl,
 																const sensor_msgs::PointCloud2::Ptr& br){
	//-- marge to local map frame --//
  sensor_msgs::PointCloud2 temp, output;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  transform = getTransform(3.41199994087, 0, 0.003);
  pcl_ros::transformPointCloud(transform, *f, temp);
  pcl::concatenatePointCloud(output, temp, output);

  transform = getTransform(2.42199993134, 0.800000011921, 1.542);
  pcl_ros::transformPointCloud(transform, *fl, temp);
  pcl::concatenatePointCloud(output, temp, output);

  transform = getTransform(2.42199993134, -0.800000011921, -1.588);
  pcl_ros::transformPointCloud(transform, *fr, temp);
  pcl::concatenatePointCloud(output, temp, output);

  transform = getTransform(-0.561999976635, 0.628000020981, 3.044);
  pcl_ros::transformPointCloud(transform, *bl, temp);
  pcl::concatenatePointCloud(output, temp, output);

  transform = getTransform(-0.561999976635, -0.617999970913, -3.074);
  pcl_ros::transformPointCloud(transform, *br, temp);
  pcl::concatenatePointCloud(output, temp, output);

	//-- transform to front radar frame --//
  transform = getTransform(-3.41199994087, 0, -0.003);
  pcl_ros::transformPointCloud(transform, output, output);

	return output;
}

void MergeCallback (const conti_radar::MeasurementConstPtr& f_input,
                    const conti_radar::MeasurementConstPtr& fl_input,
                    const conti_radar::MeasurementConstPtr& fr_input,
                    const conti_radar::MeasurementConstPtr& bl_input,
                    const conti_radar::MeasurementConstPtr& br_input) 
{
  sensor_msgs::PointCloud2::Ptr f(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fl(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fr(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr bl(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr br(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr f_i(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fl_i(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fr_i(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr bl_i(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr br_i(new sensor_msgs::PointCloud2);

  sensor_msgs::PointCloud2::Ptr f_07(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fl_07(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fr_07(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr bl_07(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr br_07(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr f_04(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fl_04(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr fr_04(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr bl_04(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr br_04(new sensor_msgs::PointCloud2);

	//cout << "front: ";
	conti_filter(f_input, f, f_i, f_07, f_04);
	//cout << "front l: ";
	conti_filter(fl_input, fl, fl_i, fl_07, fl_04);
	//cout << "front r: ";
	conti_filter(fr_input, fr, fr_i, fr_07, fr_04);
	//cout << "back l: ";
	conti_filter(bl_input, bl, bl_i, bl_07, bl_04);
	//cout << "back r: ";
	conti_filter(br_input, br, br_i, br_07, br_04);

  sensor_msgs::PointCloud2 valid_pc;
  valid_pc.header = f_input->header;
	valid_pc = merge(f, fl, fr, bl, br);
  valid_pc.header.frame_id = "/nuscenes_radar_front";

  sensor_msgs::PointCloud2 invalid_pc;
  invalid_pc.header = f_input->header;
	invalid_pc = merge(f_i, fl_i, fr_i, bl_i, br_i);
  invalid_pc.header.frame_id = "/nuscenes_radar_front";

  valid_pub.publish(valid_pc);
  invalid_pub.publish(invalid_pc);

  sensor_msgs::PointCloud2 pc_07;
  pc_07.header = f_input->header;
	pc_07 = merge(f_07, fl_07, fr_07, bl_07, br_07);
  pc_07.header.frame_id = "/nuscenes_radar_front";
  sensor_msgs::PointCloud2 pc_04;
  pc_04.header = f_input->header;
	pc_04 = merge(f_04, fl_04, fr_04, bl_04, br_04);
  pc_04.header.frame_id = "/nuscenes_radar_front";

  pub_07.publish(pc_07);
  pub_04.publish(pc_04);

	PrintTypeCount();

}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "conti_merge");
  ros::NodeHandle nh;

  message_filters::Subscriber<conti_radar::Measurement> sub_f(nh, "/radar_front", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_fl(nh, "/radar_front_left", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_fr(nh, "/radar_front_right", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_bl(nh, "/radar_back_left", 1);
  message_filters::Subscriber<conti_radar::Measurement> sub_br(nh, "/radar_back_right", 1);

  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

  no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(5), sub_f, sub_fl, sub_fr, sub_bl, sub_br);
  no_cloud_sync_->registerCallback(boost::bind(&MergeCallback, _1, _2, _3, _4, _5));

  valid_pub = nh.advertise<sensor_msgs::PointCloud2> ("radar_pc_valid", 1);
  invalid_pub = nh.advertise<sensor_msgs::PointCloud2> ("radar_pc_invalid", 1);

  pub_07 = nh.advertise<sensor_msgs::PointCloud2> ("radar_pc_invalid_0x07", 1);
  pub_04 = nh.advertise<sensor_msgs::PointCloud2> ("radar_pc_valid_0x04", 1);

  ros::spin ();
}

