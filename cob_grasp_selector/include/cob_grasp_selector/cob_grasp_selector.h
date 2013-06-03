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
 *	 This package provides service for generating collision free grasps.
 *   Takes information from the grasping request(object_ID & -- ). 
 *	 Obtains corresponding grasping possibilities from pre-generated grasps.
 *   Check all of them for collision in the planning scene. 
 *   Generates a new database of the collision-free grasps.
 *
 ****************************************************************/
#ifndef COB_GRASP_SELECTOR_H
#define COB_GRASP_SELECTOR_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <cob_grasp_validity_checker/COBGetTestGraspValidity.h>
#include <cob_grasp_selector/COBGetValidGrasp.h>
#include <cob_grasp_generator/COBNextTestGrasp.h>

class COBGraspSelector
{
private:
	ros::NodeHandle gs;
	  
	ros::ServiceClient get_next_test_grasp_client;
	ros::ServiceClient check_grasp_validity_client;
	ros::ServiceServer get_valid_grasp_server;
	
public:
	COBGraspSelector();
	void Init();
	cob_grasp_generator::COBNextTestGrasp::Response GetNextTestGrasp(cob_grasp_selector::COBGetValidGrasp::Request &req, bool new_situation);
	cob_grasp_validity_checker::COBGetTestGraspValidity::Response CheckTestGraspValidity(cob_grasp_validity_checker::COBGetTestGraspValidity::Request COBGetTestGraspValidity_req);
	sensor_msgs::JointState MapHandConfiguration(sensor_msgs::JointState table_config);
	bool ProvideValidGrasp(cob_grasp_selector::COBGetValidGrasp::Request &req, cob_grasp_selector::COBGetValidGrasp::Response &res);
	void Run();
	geometry_msgs::Pose PoseFromObjectToSDHPalmLink(geometry_msgs::Pose grasp_pose, cob_grasp_selector::COBGetValidGrasp::Request req);
};
#endif

	
	
	

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
