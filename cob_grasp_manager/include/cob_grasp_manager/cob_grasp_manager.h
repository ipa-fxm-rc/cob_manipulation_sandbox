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
#ifndef COB_GRASP_MANAGER_H
#define COB_GRASP_MANAGER_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "cob_object_detection_msgs/DetectObjects.h"
#include <cstdlib>
#include <tf/transform_listener.h>
#include <cob_grasp_selector/COBGetValidGrasp.h>
#include <cob_grasp_manager/COBGetObjectGrasp.h>
#include <cob_grasp_manager/cob_grasp_manager.h>

class COBGraspManager
{
	
private:
	ros::NodeHandle gm;
	
	ros::ServiceClient detect_object_client;
	ros::ServiceClient grasp_select_client;
	ros::ServiceServer provides_requested_object_grasp;
	
public:
	COBGraspManager();
	void Init();
	cob_object_detection_msgs::Detection GetRequestedObjectPoseInfo(cob_grasp_manager::COBGetObjectGrasp::Request req);
	cob_grasp_selector::COBGetValidGrasp::Response GetObjectInfoValidGrasp(cob_object_detection_msgs::Detection detected_object_info, cob_grasp_manager::COBGetObjectGrasp::Request req);
	bool GetRequestedObjectGrasp(cob_grasp_manager::COBGetObjectGrasp::Request &req, cob_grasp_manager::COBGetObjectGrasp::Response &res);
	void Run();
};
#endif
	
