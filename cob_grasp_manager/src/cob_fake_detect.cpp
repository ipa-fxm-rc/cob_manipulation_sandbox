#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cob_grasp_manager/cob_grasp_manager.h>
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/GetWorldProperties.h"
#include "cob_object_detection_msgs/DetectObjects.h"
#include <vector>

bool object_fake_detect(cob_object_detection_msgs::DetectObjects::Request  &req, cob_object_detection_msgs::DetectObjects::Response  &res)
{	
	ros::WallTime start = ros::WallTime::now();
	
	ros::NodeHandle m;
	ros::ServiceClient gms_c = m.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
	gazebo_msgs::GetWorldProperties getworldstate;
	gms_c.call(getworldstate);

	for(int i=0;i<(getworldstate.response.model_names.size());i++)
	{ 
		if(!strcmp(getworldstate.response.model_names[i].c_str(), req.object_name.data.c_str()))
		{
			ROS_INFO("object_found %s", req.object_name.data.c_str());
			ROS_INFO("all good1");
			ros::ServiceClient gms_d = m.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
			ROS_INFO("all good2");
			gazebo_msgs::GetModelState getmodelstate;
			getmodelstate.request.model_name=req.object_name.data;
			ROS_INFO("all good3");
			getmodelstate.request.relative_entity_name="base_footprint";
	
			//~ ros::Duration(2.0).sleep();
			gms_d.call(getmodelstate);
			ROS_INFO("all good4");

			ROS_INFO("x is %f", getmodelstate.response.pose.position.x);
			ROS_INFO("y is %f", getmodelstate.response.pose.position.y);
			ROS_INFO("z is %f", getmodelstate.response.pose.position.z);
	
	
	
			geometry_msgs::PoseStamped pBase, pMap;	
			ros::Time abhi = ros::Time::now();
	
			//~ ros::Duration(0.3).sleep();
			pBase.header.frame_id = "/base_footprint";
			pBase.pose=getmodelstate.response.pose;
			tf::TransformListener listener;
			try 
			{
				ros::Duration(2.0).sleep();
				listener.waitForTransform("/base_footprint","/head_color_camera_l_link", abhi, ros::Duration(1.0));
				listener.transformPose("/head_color_camera_l_link", pBase, pMap);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s",ex.what());
			}
			ROS_INFO("x camera is %f", pMap.pose.position.x);
			ROS_INFO("y camera is %f", pMap.pose.position.y);
			ROS_INFO("z camera is %f", pMap.pose.position.z);
			ROS_INFO("frame is %s", pMap.header.frame_id.c_str());
	
	//~ 
	//~ try 
	//~ {
		//~ ros::Duration(2.0).sleep();
		//~ listener.waitForTransform("/base_footprint","/odom_combined", abhi, ros::Duration(1.0));
		//~ listener.transformPose("/odom_combined", pBase, pMap);
	//~ }
	//~ catch (tf::TransformException &ex)
	//~ {
		//~ ROS_ERROR("%s",ex.what());
	//~ }
	//~ ROS_INFO("x odom is %f", pMap.pose.position.x);
	//~ ROS_INFO("y odom is %f", pMap.pose.position.y);
	//~ ROS_INFO("z odom is %f", pMap.pose.position.z);
	//~ ROS_INFO("frame is %s", pMap.header.frame_id.c_str());	
	
	
			cob_object_detection_msgs::Detection detection;
			detection.pose.pose=pMap.pose;
			detection.pose.header.frame_id="/head_color_camera_l_link";
			detection.label = req.object_name.data;
	//~ std::vector<int>::iterator it;
	//~ it=res.object_list.detections.begin();
	
			res.object_list.detections.insert(res.object_list.detections.begin(),detection);
		}
		else
		{ROS_INFO("object %s not found", req.object_name.data.c_str());}
	}
	
	ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    ROS_INFO("cob_fake_detect took %f seconds", dur.toSec());
	
		return true;
}
int main(int argc, char **argv)
{
ros::init(argc, argv, "my_node_test");

ros::NodeHandle n;
ros::ServiceServer service = n.advertiseService("detect_object", object_fake_detect);

ROS_INFO("fake detector ready");
ros::spin();
return 0;
}
