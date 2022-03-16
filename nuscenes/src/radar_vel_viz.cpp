#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <conti_radar/Measurement.h>

// #include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <iostream>
using namespace std;

ros::Publisher vis_pub_comp;
ros::Publisher vis_pub;
ros::Publisher vis_pub_radial;
ros::Publisher vis_pub_angular;
ros::Publisher vis_pub_self_comp;
ros::Publisher vis_pub_self_radial_static;
ros::Publisher vis_pub_self_angular_static;
ros::Publisher pub_label_id;
////////////////////////

tf::TransformListener* tran;
tf::StampedTransform transform_gt_old;
ros::Time old_t;
ros::Time label_old_t;

tf::Transform tf_radar_to_label_1;
tf::Transform tf_radar_to_label_2;
tf::Transform tf_radar_to_label_3;

void f_cb(const conti_radar::MeasurementPtr& input){
	visualization_msgs::MarkerArray marker_array_comp;
	visualization_msgs::MarkerArray marker_array;
	visualization_msgs::MarkerArray marker_array_radial;
	visualization_msgs::MarkerArray marker_array_angular;
	visualization_msgs::MarkerArray marker_array_self_comp;
	visualization_msgs::MarkerArray marker_array_radial_static;
	visualization_msgs::MarkerArray marker_array_angular_static;


	//////////////////////////////////////////////////////////////////////
	//-- caculate gt speed --//
	Eigen::Vector2d gt_car_vel(0,0); // vx, vy
	double gt_car_omega; // angular speed
  tf::StampedTransform transform_gt;
	tf::Transform tf_f_by_f; // frame by frame transformation
  try{
		tran->waitForTransform("/map", "/nuscenes_radar_front",
		                          ros::Time::now(), ros::Duration(0.07)); // 0.07 when play rate=1
    tran->lookupTransform("/map", "/nuscenes_radar_front", ros::Time(0), transform_gt);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	ros::Time curr_t = ros::Time::now();
	double delta_t = (curr_t - old_t).toSec();
	tf_f_by_f = transform_gt_old.inverse()*transform_gt;

	double delta_x = (double)tf_f_by_f.getOrigin().x();
	double delta_y = (double)tf_f_by_f.getOrigin().y();
	double delta_theta = (double)tf_f_by_f.getRotation().z();

	gt_car_vel(0) = delta_x/delta_t; // vx
	gt_car_vel(1) = delta_y/delta_t; // vy
	gt_car_omega = delta_theta/delta_t; // omega

	cout << "delta_t: " << delta_t << endl;
	cout << "gt_car_vel: " << gt_car_vel.transpose() << endl;

	transform_gt_old = transform_gt;
	old_t = curr_t;
	//////////////////////////////////////////////////////////////////////
	for(int i=0; i<input->points.size(); i++){
	if ((input->points[i].invalid_state) != 0x00){} // Do not thing
	else{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/nuscenes_radar_front";
		marker.header.stamp = ros::Time();
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
		double vel_radial_proj = vel.dot(radial_vector_normalized); // v raw projection

		theta = atan2(radial_vector_normalized(1), radial_vector_normalized(0));
		Q.setRPY( 0, 0, theta );
		marker.pose.orientation = tf2::toMsg(Q);

		marker.scale.x = vel_radial_proj; //~lenght~//
		marker.scale.y = 0.6;
		marker.scale.z = 0.6;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;

		ros::Duration d(0.1); // lifetime in sec
		marker.lifetime = d;

		marker_array_radial.markers.push_back(marker);

		// text part //
		/*
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.text = to_string(vel_radial_proj);
		marker.scale.z = 1.0;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker_array_radial.markers.push_back(marker);
		*/
		///////////////////////////////////////////////////////////////////
		// v raw angular //
		Eigen::Vector2d tangent_vector(-(input->points[i].lateral_dist), (input->points[i].longitude_dist));
		Eigen::Vector2d tangent_vector_normalized = tangent_vector.normalized();
		double vel_tangent_proj = vel.dot(tangent_vector_normalized); //projection

		theta = atan2(tangent_vector_normalized(1), tangent_vector_normalized(0));
		Q.setRPY( 0, 0, theta);
		marker.pose.orientation = tf2::toMsg(Q);

		marker.type = visualization_msgs::Marker::ARROW;
		marker.scale.x = vel_tangent_proj; //~lenght~//
		marker.scale.y = 0.6;
		marker.scale.z = 0.6;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;

		marker_array_angular.markers.push_back(marker);
		///////////////////////////////////////////////////////////////////
		// self comp v //
		double gt_car_vel_radial_proj = gt_car_vel.dot(radial_vector_normalized); // gt_car_vel projection
		double vel_radial_proj_comp = vel_radial_proj + gt_car_vel_radial_proj;

		marker.type = visualization_msgs::Marker::ARROW;
		marker.scale.x = vel_radial_proj_comp; //~lenght~//
		marker.scale.y = 0.4;
		marker.scale.z = 0.4;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 0.2;
		marker.color.b = 0.0;

		marker_array_self_comp.markers.push_back(marker);
		///////////////////////////////////////////////////////////////////
		// self v radial for static object //

		theta = atan2(radial_vector_normalized(1), radial_vector_normalized(0));
		Q.setRPY( 0, 0, theta );
		marker.pose.orientation = tf2::toMsg(Q);

		marker.type = visualization_msgs::Marker::ARROW;
		marker.scale.x = -gt_car_vel_radial_proj; //~lenght~//
		marker.scale.y = 0.4;
		marker.scale.z = 0.4;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.2;

		marker_array_radial_static.markers.push_back(marker);
		///////////////////////////////////////////////////////////////////
		// self v tangent for static object //
		double gt_car_vel_angular_proj = gt_car_vel.dot(tangent_vector_normalized);
		double r = sqrt(pow((input->points[i].lateral_dist),2)+pow((input->points[i].longitude_dist),2));//distance
		double gt_car_vel_rotation = gt_car_omega*r; // v = r*w

		theta = atan2(tangent_vector_normalized(1), tangent_vector_normalized(0));
		Q.setRPY( 0, 0, theta);
		marker.pose.orientation = tf2::toMsg(Q);

		marker.scale.x = -gt_car_vel_angular_proj-gt_car_vel_rotation; //~lenght~//
		marker.scale.y = 0.5;
		marker.scale.z = 0.5;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.8;
		marker.color.g = 0.2;
		marker.color.b = 0.0;

		marker_array_angular_static.markers.push_back(marker);
	}
	}

	//////////////////////////////////////////////////////////////////////
	vis_pub_comp.publish( marker_array_comp );
	vis_pub.publish( marker_array );
	vis_pub_radial.publish( marker_array_radial );
	vis_pub_self_comp.publish( marker_array_self_comp );
	vis_pub_self_radial_static.publish( marker_array_radial_static );
	vis_pub_angular.publish( marker_array_angular );
	vis_pub_self_angular_static.publish( marker_array_angular_static );
}

void label_cb(const visualization_msgs::MarkerArrayPtr& input){

	ros::Time curr_t = input->markers[0].header.stamp;
	double delta_t = (curr_t - label_old_t).toSec();
	cout << "delta_t: " << delta_t << endl;

	visualization_msgs::MarkerArray marker_array_id;

	for(int i=0; i<input->markers.size(); i++){
		//tf::Transform tf_lidar_to_label = tf::poseMsgToTF(input->markers[i].pose);
		//tf::Transform tf_radar_to_label = 
		visualization_msgs::Marker marker;
		marker = input->markers[i];
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		//ros::Duration d(1.0);
    //marker.lifetime = d;
		marker.text = to_string(input->markers[i].id);
		marker.scale.z = 1.0;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		marker_array_id.markers.push_back(marker);
	}
	pub_label_id.publish( marker_array_id );
///////////////////////////////////////////////////////////////////////

	tf::Transform tf_radar_front_to_lidar;
	tf_radar_front_to_lidar.setOrigin( tf::Vector3(-2.470, 0.007, 1.340) );
	tf::Quaternion q(-0.007, 0.011, -0.708, 0.706);
  //q.setRPY(0, 0, msg->theta);
  tf_radar_front_to_lidar.setRotation(q);
	for(int i=0; i<input->markers.size(); i++){
		if(input->markers[i].id == 4 || input->markers[i].id == 5 || input->markers[i].id == 6){
			tf::Transform tf_lidar_to_label;
			tf::poseMsgToTF(input->markers[i].pose, tf_lidar_to_label);
			tf::Transform tf_radar_to_label = tf_radar_front_to_lidar*tf_lidar_to_label;

			if(input->markers[i].id == 4){
				Eigen::Vector2d radial_vector(-(tf_radar_to_label.getOrigin().x()),
																		  -(tf_radar_to_label.getOrigin().y()));
				Eigen::Vector2d radial_vector_normalized = radial_vector.normalized();
				Eigen::Vector2d vel(tf_radar_to_label.getOrigin().x()-tf_radar_to_label_1.getOrigin().x(),
													  tf_radar_to_label.getOrigin().y()-tf_radar_to_label_1.getOrigin().y());
				double vel_radial_proj = (vel/delta_t).dot(radial_vector_normalized);
				cout << "id:4 vel_radial_proj: " << vel_radial_proj << endl;

				tf_radar_to_label_1 = tf_radar_to_label;
			}
			if(input->markers[i].id == 5){
				Eigen::Vector2d radial_vector(-(tf_radar_to_label.getOrigin().x()),
																		  -(tf_radar_to_label.getOrigin().y()));
				Eigen::Vector2d radial_vector_normalized = radial_vector.normalized();
				Eigen::Vector2d vel(tf_radar_to_label.getOrigin().x()-tf_radar_to_label_2.getOrigin().x(),
													  tf_radar_to_label.getOrigin().y()-tf_radar_to_label_2.getOrigin().y());
				double vel_radial_proj = (vel/delta_t).dot(radial_vector_normalized);
				cout << "id:5 vel_radial_proj: " << vel_radial_proj << endl;

				tf_radar_to_label_2 = tf_radar_to_label;
			}
			if(input->markers[i].id == 6){
				Eigen::Vector2d radial_vector(-(tf_radar_to_label.getOrigin().x()),
																		  -(tf_radar_to_label.getOrigin().y()));
				Eigen::Vector2d radial_vector_normalized = radial_vector.normalized();
				Eigen::Vector2d vel(tf_radar_to_label.getOrigin().x()-tf_radar_to_label_3.getOrigin().x(),
													  tf_radar_to_label.getOrigin().y()-tf_radar_to_label_3.getOrigin().y());
				double vel_radial_proj = (vel/delta_t).dot(radial_vector_normalized);
				cout << "id:6 vel_radial_proj: " << vel_radial_proj << endl;

				tf_radar_to_label_3 = tf_radar_to_label;
			}
		cout << endl;
		}
	}
  label_old_t = curr_t;
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "radar_viz");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/radar_front", 1, f_cb);
  //ros::Subscriber sub_label = nh.subscribe ("/lidar_label", 1, label_cb);

  vis_pub_comp = nh.advertise<visualization_msgs::MarkerArray> ("rf_v_comp", 1);
  vis_pub = nh.advertise<visualization_msgs::MarkerArray> ("rf_v", 1);
  vis_pub_radial = nh.advertise<visualization_msgs::MarkerArray> ("rf_v_radial", 1);
  vis_pub_angular = nh.advertise<visualization_msgs::MarkerArray> ("rf_v_angular", 1);
  vis_pub_self_comp = nh.advertise<visualization_msgs::MarkerArray> ("rf_v_self_comp", 1);
  vis_pub_self_radial_static = nh.advertise<visualization_msgs::MarkerArray> ("rf_v_self_radial_static", 1);
  vis_pub_self_angular_static = nh.advertise<visualization_msgs::MarkerArray> ("rf_v_self_angular_static", 1);
  pub_label_id = nh.advertise<visualization_msgs::MarkerArray> ("label_id", 1);

	tf::TransformListener lr(ros::Duration(10));
  tran=&lr;

  ros::spin ();
}

