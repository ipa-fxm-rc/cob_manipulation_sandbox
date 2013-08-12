#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "cob_object_detection_msgs/DetectObjects.h"
#include <cstdlib>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "detect_object1");
    ros::NodeHandle gm;
    ros::ServiceClient client = gm.serviceClient<cob_object_detection_msgs::DetectObjects>("detect_object");
    ROS_INFO("step 1");
	cob_object_detection_msgs::DetectObjects srv;
	srv.request.object_name.data="salt";
	if (client.call(srv))
	{
		ROS_INFO("step 4");
		ROS_INFO("no of detections are %d", srv.response.object_list.detections.size());
		ROS_INFO("label %s",srv.response.object_list.detections.at(0).label.c_str());
		
		//~ geometry_msgs::PoseStamped pBase, pMap;	
		//~ pBase=srv.response.object_list.detections.at(0).pose;
		//~ 
		//~ ros::Time abhi = ros::Time::now();
		//~ tf::TransformListener listener;
		//~ try 
		//~ {
			//~ ros::Duration(2.0).sleep();
			//~ listener.waitForTransform("/head_color_camera_l_link","/odom_combined", abhi, ros::Duration(1.0));
			//~ listener.transformPose("/odom_combined", pBase, pMap);
		//~ }
		//~ catch (tf::TransformException &ex)
		//~ {
			//~ ROS_ERROR("%s",ex.what());
		//~ }
		//~ ROS_INFO("x  %f", pMap.pose.position.x);
		//~ ROS_INFO("y  %f", pMap.pose.position.y);
		//~ ROS_INFO("z  %f", pMap.pose.position.z);
	}
	else {ROS_INFO("something is wrong");}
	return 0;
}
	
