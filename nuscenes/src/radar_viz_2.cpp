#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <conti_radar/Measurement.h>

#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <iostream>
using namespace std;

ros::Publisher vis_pub_comp;
ros::Publisher vis_pub;
ros::Publisher vis_pub_radial;
ros::Publisher vis_pub_angular;

std::string radar_frame;

void f_cb(const conti_radar::MeasurementPtr& input){

	visualization_msgs::MarkerArray marker_array_comp;
	visualization_msgs::MarkerArray marker_array;
	visualization_msgs::MarkerArray marker_array_radial;
	visualization_msgs::MarkerArray marker_array_angular;

	for(int i=0; i<input->points.size(); i++){
	//if ((input->points[i].invalid_state) == 0x00){ // only show valid point
		//int t = input->points[i].id;
		//cout <<"t: "<<t<<endl;

		visualization_msgs::Marker marker;
		marker.header = input->header; // marker.header.stamp = ros::Time();
		marker.header.frame_id = radar_frame;
		//marker.ns = "my_namespace";
		marker.id = i;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = input->points[i].longitude_dist;
		marker.pose.position.y = input->points[i].lateral_dist;
		marker.pose.position.z = 0;
		///////////////////////////////////////////////////////////////////
		// v comp //
		float theta = atan2(input->points[i].lateral_vel_comp,
												input->points[i].longitude_vel_comp);
		tf2::Quaternion Q;
		Q.setRPY( 0, 0, theta );
		marker.pose.orientation = tf2::toMsg(Q);

		marker.scale.x = sqrt(pow(input->points[i].lateral_vel_comp,2) + 
				 									pow(input->points[i].longitude_vel_comp,2)); //~lenght~//
		marker.scale.y = 0.6;
		marker.scale.z = 0.6;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		// marker.lifetime = 

		marker_array_comp.markers.push_back(marker);
		///////////////////////////////////////////////////////////////////
		// v raw //
		theta = atan2(input->points[i].lateral_vel, \
									input->points[i].longitude_vel);
		Q.setRPY( 0, 0, theta );
		marker.pose.orientation = tf2::toMsg(Q);

		marker.scale.x = sqrt(pow(input->points[i].lateral_vel,2) + 
				 									pow(input->points[i].longitude_vel,2)); //~lenght~//
		marker.scale.y = 0.6;
		marker.scale.z = 0.6;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;

		marker_array.markers.push_back(marker);
		///////////////////////////////////////////////////////////////////
		// v raw radial //
		Eigen::Vector2d radial_vector(-(input->points[i].longitude_dist), -(input->points[i].lateral_dist));
		Eigen::Vector2d radial_vector_normalized = radial_vector.normalized();
		Eigen::Vector2d vel(input->points[i].longitude_vel, input->points[i].lateral_vel);
		double vel_radial_proj = vel.dot(radial_vector_normalized); //projection

		theta = atan2(radial_vector_normalized(1), radial_vector_normalized(0));
		Q.setRPY( 0, 0, theta );
		marker.pose.orientation = tf2::toMsg(Q);

		marker.scale.x = vel_radial_proj; //~lenght~//
		marker.scale.y = 0.45;
		marker.scale.z = 0.45;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		ros::Duration d(0.3); // lifetime in sec
		marker.lifetime = d;

		marker_array_radial.markers.push_back(marker);
		///////////////////////////////////////////////////////////////////
		// v raw angular //
		Eigen::Vector2d tangent_vector(-(input->points[i].lateral_dist), (input->points[i].longitude_dist));
		Eigen::Vector2d tangent_vector_normalized = tangent_vector.normalized();
		double vel_tangent_proj = vel.dot(tangent_vector_normalized); //projection

		theta = atan2(tangent_vector_normalized(1), tangent_vector_normalized(0));
		Q.setRPY( 0, 0, theta);
		marker.pose.orientation = tf2::toMsg(Q);

		marker.scale.x = vel_tangent_proj; //~lenght~//
		marker.scale.y = 0.6;
		marker.scale.z = 0.6;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;

		marker_array_angular.markers.push_back(marker);
	//}
	}

	//////////////////////////////////////////////////////////////////////
	// vis_pub_comp.publish( marker_array_comp );
	// vis_pub.publish( marker_array );
	vis_pub_radial.publish( marker_array_radial );
	// vis_pub_angular.publish( marker_array_angular );
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "radar_viz");
  ros::NodeHandle nh("~");

  if (nh.getParam("radar_frame", radar_frame))
  {
    ROS_INFO("Got 'radar_frame' param: %s", radar_frame.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get param 'radar_frame'");
  }

  ros::Subscriber sub = nh.subscribe ("/radar_topic", 1, f_cb); // radar_front

  vis_pub_comp = nh.advertise<visualization_msgs::MarkerArray> ("v_comp", 1); //rf_v_comp
  vis_pub = nh.advertise<visualization_msgs::MarkerArray> ("v", 1); //rf_v
  vis_pub_radial = nh.advertise<visualization_msgs::MarkerArray> ("v_radial", 1); //rf_v_radial
  vis_pub_angular = nh.advertise<visualization_msgs::MarkerArray> ("v_angular", 1); //rf_v_angular

  ros::spin ();
}

