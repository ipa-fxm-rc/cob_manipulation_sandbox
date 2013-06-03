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
 * This package generates grasp for a particular object ID. It takes grasp information
 * from a Grasp-Table generated  offline. It returns Grasp pose with respect to the object.
 *
 ****************************************************************/
#ifndef COB_GRASP_VALIDITY_CHECKER_H
#define COB_GRASP_VALIDITY_CHECKER_H
 
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <planning_environment/models/collision_models.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <cob_grasp_validity_checker/COBGetTestGraspValidity.h>

class COBGraspValidityChecker
{
private:
	ros::NodeHandle rh;
	ros::ServiceServer grasp_validity_server;
	ros::Publisher vis_marker_publisher_;
	ros::Publisher vis_marker_array_publisher_;
	tf::TransformListener listener;
	ros::ServiceClient get_planning_scene_client;
	ros::ServiceClient set_planning_scene_diff_client;
	
public:
	planning_environment::CollisionModels collision_models;
	COBGraspValidityChecker();
	void Init();
	bool COBGetGraspValidity(cob_grasp_validity_checker::COBGetTestGraspValidity::Request &req,cob_grasp_validity_checker::COBGetTestGraspValidity::Response &res);
	planning_models::KinematicState* COBGraspGetSetPlanningScene(cob_grasp_validity_checker::COBGetTestGraspValidity::Request &req);
	bool COBGraspVisualizeValidate(planning_models::KinematicState* state);
	geometry_msgs::PoseStamped TransformPoseToDesiredFrame(cob_grasp_validity_checker::COBGetTestGraspValidity::Request &req);
	arm_navigation_msgs::GetPlanningScene::Response COBGraspGetPlanningScene(arm_navigation_msgs::GetPlanningScene::Request planning_scene_req, arm_navigation_msgs::GetPlanningScene::Response planning_scene_res);

	void Run();
};
		
#endif
