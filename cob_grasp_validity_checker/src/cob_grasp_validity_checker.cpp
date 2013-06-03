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
 * \date Date of creation: March, 2013
 *
 * \brief
 *   This package checks the validity of the grasp provided.
 * 	 It takes pose of the grasp and checks for collision and returns 
 *   the validity data for the grasp for the defined planning scene.
 *
 ****************************************************************/
#include <iostream>
#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <cob_grasp_validity_checker/cob_grasp_validity_checker.h>
#include <geometric_shapes/shape_operations.h>
#include <planning_environment/util/construct_object.h>

using namespace std;

COBGraspValidityChecker::COBGraspValidityChecker():collision_models("robot_description")
	{}

void COBGraspValidityChecker::Init()
{
	vis_marker_publisher_ = rh.advertise<visualization_msgs::Marker>("state_validity_markers", 128);
	vis_marker_array_publisher_ = rh.advertise<visualization_msgs::MarkerArray>("state_validity_markers_array", 128);
	grasp_validity_server = rh.advertiseService("grasp_validity_server", &COBGraspValidityChecker::COBGetGraspValidity, this);
	static const std::string GET_PLANNING_SCENE_NAME = "/environment_server/get_planning_scene";
    static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
	get_planning_scene_client = rh.serviceClient<arm_navigation_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);
	set_planning_scene_diff_client = rh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
	
}

arm_navigation_msgs::GetPlanningScene::Response COBGraspValidityChecker::COBGraspGetPlanningScene(arm_navigation_msgs::GetPlanningScene::Request planning_scene_req, arm_navigation_msgs::GetPlanningScene::Response planning_scene_res)
{
	
	return planning_scene_res;
}

bool COBGraspValidityChecker::COBGetGraspValidity(cob_grasp_validity_checker::COBGetTestGraspValidity::Request &req,cob_grasp_validity_checker::COBGetTestGraspValidity::Response &res)
{
	ros::WallTime start = ros::WallTime::now();
//	
	planning_models::KinematicState* state;
//	
	ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    if(dur.toSec()>1)
	{ROS_INFO("1 %f seconds for 1 grasp", dur.toSec());}
//	
	state=COBGraspGetSetPlanningScene(req);	
//	
	end = ros::WallTime::now();
    dur = end - start;
	if(dur.toSec()>1)
	{ROS_INFO("2 %f seconds for 1 grasp", dur.toSec());}
//    
	res.success.data=COBGraspVisualizeValidate(state);
//	
	end = ros::WallTime::now();
    dur = end - start;
    if(dur.toSec()>1)
	{ROS_INFO("3 %f seconds for 1 grasp", dur.toSec());}
    ROS_INFO("Grasp Validity Checker took %f seconds for 1 grasp", dur.toSec());
    
	return true;
}

planning_models::KinematicState* COBGraspValidityChecker::COBGraspGetSetPlanningScene(cob_grasp_validity_checker::COBGetTestGraspValidity::Request &req)
{		
	ros::WallTime start = ros::WallTime::now();
	arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
	arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

	if(!get_planning_scene_client.call(planning_scene_req, planning_scene_res)) 
	{
		ROS_WARN("Can't get planning scene");
	}

	arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_diff_req;
	arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_diff_res;

	planning_scene_diff_req.planning_scene_diff = planning_scene_res.planning_scene;

	planning_scene_diff_req.planning_scene_diff.robot_state.multi_dof_joint_state.poses[1]=req.pose;
	
	ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    if(dur.toSec()>1)
	{ROS_INFO("COBGraspGetSetPlanningScene_1 %f seconds for 1 grasp", dur.toSec());}
	

	std::vector<std::string> def_joint_names;
	def_joint_names.push_back("sdh_knuckle_joint");
	def_joint_names.push_back("sdh_thumb_2_joint");
	def_joint_names.push_back("sdh_thumb_3_joint");
	def_joint_names.push_back("sdh_finger_12_joint");
	def_joint_names.push_back("sdh_finger_13_joint");
	def_joint_names.push_back("sdh_finger_22_joint");
	def_joint_names.push_back("sdh_finger_23_joint");
	std::string comparison_joint;
	
	for(int i=0;i<sizeof(planning_scene_diff_req.planning_scene_diff.robot_state.joint_state.name);i++)
	{ 
		if(!strcmp(planning_scene_diff_req.planning_scene_diff.robot_state.joint_state.name[i].c_str(), "sdh_finger_21_joint"))
		{
			planning_scene_diff_req.planning_scene_diff.robot_state.joint_state.position[i]=req.joint_position.position[0];
		}
		
		for(int j=0;j< 7;j++)
		{
			comparison_joint = planning_scene_diff_req.planning_scene_diff.robot_state.joint_state.name[i];
			if (!strcmp(comparison_joint.c_str(), def_joint_names[j].c_str()))
			{
				planning_scene_diff_req.planning_scene_diff.robot_state.joint_state.position.at(i) = req.joint_position.position[j];
				//ROS_INFO("%s position is: %f", comparison_joint.c_str(), req.joint_position.position[j]);
			}
		}
	}	
	
	end = ros::WallTime::now();
    dur = end - start;
    if(dur.toSec()>1)
	{ROS_INFO("COBGraspGetSetPlanningScene_2 %f seconds for 1 grasp", dur.toSec());}
	
	if(set_planning_scene_diff_client.call(planning_scene_diff_req, planning_scene_diff_res)) 
	{
		ROS_INFO("Set planning scene:: DONE");
	}
	else
	{
		ROS_WARN("Can't set planning scene");
	}
	end = ros::WallTime::now();
    dur = end - start;
    if(dur.toSec()>1)
	{ROS_INFO("COBGraspGetSetPlanningScene_3 %f seconds for 1 grasp", dur.toSec());}
	planning_models::KinematicState* state = collision_models.setPlanningScene(planning_scene_diff_req.planning_scene_diff);
	
	end = ros::WallTime::now();
    dur = end - start;
    if(dur.toSec()>1)
	{ROS_INFO("COBGraspGetSetPlanningScene_4 %f seconds for 1 grasp", dur.toSec());}
	
	return state;
}
bool COBGraspValidityChecker::COBGraspVisualizeValidate(planning_models::KinematicState* state)
{
	bool validity;
	std_msgs::ColorRGBA good_color, collision_color, joint_limits_color;
	good_color.a = collision_color.a = joint_limits_color.a = .8;
	good_color.g = 1.0;
	collision_color.r = 1.0;
	joint_limits_color.b = 1.0;
	
	std_msgs::ColorRGBA point_markers;
	point_markers.a = 1.0;
	point_markers.r = 1.0;
	point_markers.g = .8;
	validity=false;
	
	std::vector<std::string> joint_names;
	std::vector<std::string> arm_names;
	std_msgs::ColorRGBA color;	
	visualization_msgs::MarkerArray arr;
	


	arm_names = collision_models.getKinematicModel()->getModelGroup("sdh_grasp")->getUpdatedLinkModelNames();
	joint_names = collision_models.getKinematicModel()->getModelGroup("sdh_grasp")->getJointModelNames();
	
	if(!state->areJointsWithinBounds(joint_names))
	{
		color = joint_limits_color;
	}
	else if(collision_models.isKinematicStateInCollision(*state)) 
	{
		color = collision_color;
		collision_models.getAllCollisionPointMarkers(*state,
													arr,
													point_markers,
													ros::Duration(0.2));
	}
	else
	{
		color = good_color;
		validity=true;
	}
	collision_models.getRobotMarkersGivenState(*state,
												arr,
												color,
												"sdh_grasp",
												ros::Duration(0.2),
												&arm_names);

	//while(ros::ok()) 
	//{    
		vis_marker_array_publisher_.publish(arr);
		ros::spinOnce();
		ros::Duration(.1).sleep();
	//}
	collision_models.revertPlanningScene(state);
	
	return validity;
}


void COBGraspValidityChecker::Run()
{
	ROS_DEBUG("GraspValidityChecker::Run(): ");
	
	ROS_INFO("cob_grasp_validity_checker...spinning");
    ros::spin();
}	


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_grasp_validity_checker");
	COBGraspValidityChecker*  cob_grasp_validity_checker= new COBGraspValidityChecker();
	cob_grasp_validity_checker->Init();
	cob_grasp_validity_checker->Run();

	return 0;
}
