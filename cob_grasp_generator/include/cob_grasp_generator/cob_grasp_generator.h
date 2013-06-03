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
#ifndef __COB_GRASP_GENERATOR_H__
#define __COB_GRASP_GENERATOR_H__

#include <ros/ros.h>
#include <GraspTable.h>
#include <cob_grasp_generator/GraspDefinition.h>
#include <cob_grasp_generator/COBNextTestGrasp.h>
#include "geometry_msgs/Quaternion.h"
#include <tf/tf.h>


class GetNextTestGrasp
{
private:

    ros::NodeHandle gg;
	unsigned int grasp_index;
	char* GraspTableIniFile;
	ros::ServiceServer grasp_generator_server;
	GraspTable* m_GraspTable;

public:

	bool COBProvideNextGrasp(cob_grasp_generator::COBNextTestGrasp::Request &req, cob_grasp_generator::COBNextTestGrasp::Response &res);
	geometry_msgs::Quaternion ConvertRPYtoQuaternion(std::vector<double> pose)
	{
	geometry_msgs::Quaternion q;
	q=tf::createQuaternionMsgFromRollPitchYaw(pose[3],pose[4],pose[5]);	
	return q;
	}
	void Init();
	void Run();
};

#endif
