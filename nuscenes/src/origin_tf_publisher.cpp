#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "tf_listener");
  ros::NodeHandle node;
  tf::TransformListener listener;
	static tf::TransformBroadcaster br;
	tf::StampedTransform transform_map_to_origin;
	bool init = false;
  ros::Rate rate(200.0);

  while (node.ok()){
		if(!init){
		  try{
				listener.waitForTransform("/map", "/car",
				                          ros::Time::now(), ros::Duration(0.1));
		    listener.lookupTransform("/map", "/car",  
		                             ros::Time(0), transform_map_to_origin);
				init = true;
		  }
		  catch (tf::TransformException ex){
		    ROS_WARN("%s",ex.what());
		    ros::Duration(1.0).sleep();
		  }
		}
  	br.sendTransform(tf::StampedTransform(transform_map_to_origin, ros::Time::now(), "/map", "/origin"));
    rate.sleep();
  }

  return 0;
};
