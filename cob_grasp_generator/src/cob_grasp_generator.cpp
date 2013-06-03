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

#include <ros/ros.h>
#include <cob_grasp_generator/cob_grasp_generator.h> 


 
void GetNextTestGrasp::Init()
{
	grasp_index = 0;
	GraspTableIniFile="/home/fxm-rc/git/care-o-bot/cob_manipulation_sandbox/cob_grasp_generator/files/GraspTable.txt";
    grasp_generator_server = gg.advertiseService("get_next_grasp_server", &GetNextTestGrasp::COBProvideNextGrasp, this);
    m_GraspTable = new GraspTable();
    int error = m_GraspTable->Init(GraspTableIniFile);
}


bool GetNextTestGrasp::COBProvideNextGrasp(cob_grasp_generator::COBNextTestGrasp::Request &req, cob_grasp_generator::COBNextTestGrasp::Response &res)
{
	ros::WallTime start = ros::WallTime::now();
	
	unsigned int objectClassId=req.objectID;
	res.grasp.grasp_available.data=1;
	Grasp * current_grasp = NULL;
	if(req.new_situation.data){grasp_index = 0;}
	current_grasp = m_GraspTable->GetNextGrasp(objectClassId, req.new_situation.data);
	if (current_grasp == NULL)
	{
		res.grasp.grasp_available.data=0;
	}
	else 
	{
		grasp_index=grasp_index+1;
		ROS_INFO("index is %d",grasp_index);
		res.grasp.index=grasp_index;
	
		std::vector<double> current_hand_pre_pose = current_grasp->GetTCPPreGraspPose();
		res.grasp.pre_grasp_pose.orientation=ConvertRPYtoQuaternion(current_hand_pre_pose);
	
		res.grasp.pre_grasp_pose.position.x=current_hand_pre_pose[0];
		res.grasp.pre_grasp_pose.position.y=current_hand_pre_pose[1];
		res.grasp.pre_grasp_pose.position.z=current_hand_pre_pose[2];
		
		std::vector<double> current_hand_pre_config = current_grasp->GetHandPreGraspConfig();
		
		for (int ii=0; ii< current_hand_pre_config.size();ii++)
		{
			res.grasp.hand_pre_grasp_config.position.push_back(current_hand_pre_config[ii]);
		}	
	
		std::vector<double> current_hand_pose = current_grasp->GetTCPGraspPose();
		res.grasp.grasp_pose.orientation=ConvertRPYtoQuaternion(current_hand_pose);	
		
		res.grasp.grasp_pose.position.x=current_hand_pose[0];
		res.grasp.grasp_pose.position.y=current_hand_pose[1];
		res.grasp.grasp_pose.position.z=current_hand_pose[2];
	
		std::vector<double> current_hand_config = current_grasp->GetHandGraspConfig();	
		for (int ii=0; ii< current_hand_config.size();ii++)
		{
			res.grasp.hand_grasp_config.position.push_back(current_hand_config[ii]);
		}

		std::vector<double> current_hand_optimal_config = current_grasp->GetHandOptimalGraspConfig();	
		for (int ii=0; ii< current_hand_optimal_config.size();ii++)
		{
			res.grasp.hand_optimal_grasp_config.position.push_back(current_hand_optimal_config[ii]);
		}	
	}
	
	ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    ROS_INFO("Grasp Geneorator took %f seconds", dur.toSec());
    
	return true;
}


	
	
	
void GetNextTestGrasp::Run()
{
	ROS_INFO("cob_grasp_generator...spinning");
    ros::spin();
}	


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_grasp_generator");

	GetNextTestGrasp* get_next_test_grasp=new GetNextTestGrasp();
	get_next_test_grasp->Init();
	get_next_test_grasp->Run();

	return 0;
}

