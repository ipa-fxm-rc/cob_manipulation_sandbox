/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2013 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_manipulation_sandbox
 * \note
 *   ROS package name: cob_grasp_validity_checker
 *
 * \author
 *   Author: Rohit Chandra, email:rohit.chandra@ipa.fhg.de
 *
 * \date Date of creation: April, 2013
 *
 * \brief
 *   
 *   This package gets the object information from COB Grasp Manager and 
 *   provides COB Grasp Generator with the object ID and requests for
 *   next grasp information from the Grasp Table already generated.
 *   Then it provides COB Grasp Validity Checker with the grasp information
 *   and object information and request for the validtiy information.
 *   Then the grasp information for valid grasp(if obtained) is passed to
 *   COB Grasp Manager.
 *
 ****************************************************************/
 
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cob_grasp_manager/cob_grasp_manager.h>

using namespace std;

COBGraspManager::COBGraspManager(){}

void COBGraspManager::Init()
{
	detect_object_client = gm.serviceClient<cob_object_detection_msgs::DetectObjects>("detect_object");
	grasp_select_client = gm.serviceClient<cob_grasp_selector::COBGetValidGrasp>("get_valid_grasp_server");
	ros::service::waitForService("detect_object");
	ros::service::waitForService("get_valid_grasp_server");
	provides_requested_object_grasp = gm.advertiseService("object_grasp_provider", &COBGraspManager::GetRequestedObjectGrasp, this);
}

cob_object_detection_msgs::Detection COBGraspManager::GetRequestedObjectPoseInfo(cob_grasp_manager::COBGetObjectGrasp::Request req)
{
	cob_object_detection_msgs::DetectObjects srv;
	srv.request.object_name.data=req.object_name;
	ROS_INFO("Requested object is %s",req.object_name.c_str());
	cob_object_detection_msgs::Detection detected_object_info;
	
	if (detect_object_client.call(srv))
	{
		ROS_INFO("detec no. = %d", srv.response.object_list.detections.size());
		//~ 
		if (srv.response.object_list.detections.size())
		{
			ROS_INFO("number of %s found is: %d ",srv.response.object_list.detections.at(0).label.c_str(),srv.response.object_list.detections.size());
			
			geometry_msgs::PoseStamped pBase, pMap;	
			pBase=srv.response.object_list.detections.at(0).pose;
		
			tf::TransformListener listener;
			try 
			{
				ros::Duration(2.0).sleep();
				listener.waitForTransform("/head_color_camera_l_link","/sdh_palm_link", ros::Time::now(), ros::Duration(1.0));
				listener.transformPose("/sdh_palm_link", pBase, pMap);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s",ex.what());
			}
			ROS_INFO("object pose:position x %f", pMap.pose.position.x);
			ROS_INFO("object pose:position y %f", pMap.pose.position.y);
			ROS_INFO("object pose:position z %f", pMap.pose.position.z);
			ROS_INFO("object pose:orientation x %f", pMap.pose.orientation.x);
			ROS_INFO("object pose:orientation y %f", pMap.pose.orientation.y);
			ROS_INFO("object pose:orientation z %f", pMap.pose.orientation.z);
			ROS_INFO("object pose:orientation w %f", pMap.pose.orientation.w);
			
		
			detected_object_info=srv.response.object_list.detections.at(0);
			detected_object_info.pose=pMap;
		}
		else{ROS_ERROR("%s not found",req.object_name.c_str());}
	}
	else 
	{
		ROS_ERROR("Object-Detection Server call failed");
	}
	return detected_object_info;
}

cob_grasp_selector::COBGetValidGrasp::Response COBGraspManager::GetObjectInfoValidGrasp(cob_object_detection_msgs::Detection detected_object_info, cob_grasp_manager::COBGetObjectGrasp::Request req)
{
	//match the object label to object Id here or anywhere
	cob_grasp_selector::COBGetValidGrasp srv;
	srv.request.new_situation=req.new_situation;
	srv.request.objectID=13;
	srv.request.object_pose=detected_object_info.pose.pose;
	cob_grasp_selector::COBGetValidGrasp::Response Valid_Grasp_Info;
	if (grasp_select_client.call(srv))
	{	
		Valid_Grasp_Info=srv.response;
		
		if (srv.response.success.data)
		{
			ROS_INFO("Valid Grasp Found");			
		}
		else
		{
			ROS_INFO("There are no valid grasp");
		}
	}  
	else
	{
		ROS_ERROR("COB-Grasp-Selector server called failed");
	}
	return Valid_Grasp_Info;
}

bool COBGraspManager::GetRequestedObjectGrasp(cob_grasp_manager::COBGetObjectGrasp::Request &req, cob_grasp_manager::COBGetObjectGrasp::Response &res)
{
	ros::WallTime start = ros::WallTime::now();
	
	cob_object_detection_msgs::Detection detected_object_info;
	detected_object_info= GetRequestedObjectPoseInfo(req);
	
	cob_grasp_selector::COBGetValidGrasp::Response valid_grasp_info;
	valid_grasp_info=GetObjectInfoValidGrasp(detected_object_info, req);
	res.success=valid_grasp_info.success;
	res.valid_grap_pose=valid_grasp_info.valid_grap_pose;
	res.valid_pre_grap_pose=valid_grasp_info.valid_pre_grap_pose;
	res.valid_pre_grasp_joint_position=valid_grasp_info.valid_pre_grasp_joint_position;
	res.valid_grasp_joint_position=valid_grasp_info.valid_grasp_joint_position;
	res.valid_optimal_grasp_joint_position=valid_grasp_info.valid_optimal_grasp_joint_position;
	res.valid_grasp_index=valid_grasp_info.valid_grasp_index;

	ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    ROS_INFO("cob_grasp_manager took %f seconds", dur.toSec());
	
	return true;
}

void COBGraspManager::Run()
{	
	ROS_INFO("cob_grasp_manager...spinning");
    ros::spin();
}	

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "cob_grasp_manager");
    ros::NodeHandle gm;

	COBGraspManager*  cob_grasp_manager= new COBGraspManager();
	cob_grasp_manager->Init();
	cob_grasp_manager->Run();
	
	return 0;
}
	
