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

#include <cob_grasp_selector/cob_grasp_selector.h>


using namespace std;

COBGraspSelector::COBGraspSelector(){}

void COBGraspSelector::Init()
{
	get_next_test_grasp_client = gs.serviceClient<cob_grasp_generator::COBNextTestGrasp>("get_next_grasp_server");
	check_grasp_validity_client = gs.serviceClient<cob_grasp_validity_checker::COBGetTestGraspValidity>("grasp_validity_server");
	ros::service::waitForService("get_next_grasp_server");
	ros::service::waitForService("grasp_validity_server");
	
	get_valid_grasp_server = gs.advertiseService("get_valid_grasp_server", &COBGraspSelector::ProvideValidGrasp, this);
	
}

cob_grasp_generator::COBNextTestGrasp::Response COBGraspSelector::GetNextTestGrasp(cob_grasp_selector::COBGetValidGrasp::Request &req, bool new_situation)
{
	cob_grasp_generator::COBNextTestGrasp::Request COBNextTestGrasp_req;
	cob_grasp_generator::COBNextTestGrasp::Response COBNextTestGrasp_res;
	COBNextTestGrasp_req.objectID=req.objectID;
	COBNextTestGrasp_req.new_situation.data=new_situation;
	if (!get_next_test_grasp_client.call(COBNextTestGrasp_req,COBNextTestGrasp_res))
	{
		ROS_INFO("COB-Grasp-Generator Server call failed");
	}	
	return COBNextTestGrasp_res;
}


cob_grasp_validity_checker::COBGetTestGraspValidity::Response COBGraspSelector::CheckTestGraspValidity(cob_grasp_validity_checker::COBGetTestGraspValidity::Request COBGetTestGraspValidity_req)
{
	cob_grasp_validity_checker::COBGetTestGraspValidity::Response COBGetTestGraspValidity_res;
	if(!check_grasp_validity_client.call(COBGetTestGraspValidity_req, COBGetTestGraspValidity_res))
	{
		ROS_INFO("COB-Grasp-Validity-Checker Server call failed");
	}
	return COBGetTestGraspValidity_res;
}

void COBGraspSelector::Run()
{	
	ROS_INFO("cob_grasp_selector...spinning");
    ros::spin();
}	

sensor_msgs::JointState COBGraspSelector::MapHandConfiguration(sensor_msgs::JointState table_config)
{
	sensor_msgs::JointState grasp_configuration;
	grasp_configuration.position.resize(7);
	grasp_configuration.position[0]= table_config.position[0];
	grasp_configuration.position[1]= table_config.position[2];
	grasp_configuration.position[2]= table_config.position[3];
	grasp_configuration.position[3]= table_config.position[5];
	grasp_configuration.position[4]= table_config.position[6];
	grasp_configuration.position[5]= table_config.position[11];
	grasp_configuration.position[6]= table_config.position[12];
//#################################################################################################################################################################################
//                   Joint position brought to 1.57079 from 1.5708 to get proper joint limit check
//#################################################################################################################################################################################
	for(int ik=0; ik<grasp_configuration.position.size();ik++)
	{
		if(grasp_configuration.position[ik]>1.57079){grasp_configuration.position[ik]=1.57079;}
		if(grasp_configuration.position[ik]<-1.57079){grasp_configuration.position[ik]=-1.57079;}
	}
	return grasp_configuration;
}



//#########################################################################################################################################################################
//                   Transform pose from object to odom
//#########################################################################################################################################################################
geometry_msgs::Pose COBGraspSelector::PoseFromObjectToSDHPalmLink(geometry_msgs::Pose grasp_pose, cob_grasp_selector::COBGetValidGrasp::Request req)
{
	tf::Quaternion tf_Quaternion;
	tf::quaternionMsgToTF(req.object_pose.orientation,tf_Quaternion);
	tf::Vector3 tf_vector_3= tf::Vector3(req.object_pose.position.x, req.object_pose.position.y, req.object_pose.position.z);
	tf::Transform transformation_object_to_SDHPalmLink = tf::Transform(tf_Quaternion, tf_vector_3);
	
	tf::quaternionMsgToTF(grasp_pose.orientation,tf_Quaternion);
	tf_vector_3= tf::Vector3(grasp_pose.position.x/1000, grasp_pose.position.y/1000, grasp_pose.position.z/1000);
	tf::Transform transformation_grasp_to_object = tf::Transform(tf_Quaternion, tf_vector_3);
	
	
    tf::Transform transformation_grasp_to_odom_combined= transformation_object_to_SDHPalmLink.operator*(transformation_grasp_to_object );
    
    geometry_msgs::Transform msg;
    tf::transformTFToMsg (transformation_grasp_to_odom_combined,msg );
	
	geometry_msgs::Pose transformed_FromObject_ToOdom_combined_grasp_pose;
	transformed_FromObject_ToOdom_combined_grasp_pose.position.x=msg.translation.x;
	transformed_FromObject_ToOdom_combined_grasp_pose.position.y=msg.translation.y;
	transformed_FromObject_ToOdom_combined_grasp_pose.position.z=msg.translation.z;

	transformed_FromObject_ToOdom_combined_grasp_pose.orientation=msg.rotation;
	return transformed_FromObject_ToOdom_combined_grasp_pose;
	
}
//########################################################################################
//########################################################################################

bool COBGraspSelector::ProvideValidGrasp(cob_grasp_selector::COBGetValidGrasp::Request &req, cob_grasp_selector::COBGetValidGrasp::Response &res)
{	
	ros::WallTime start = ros::WallTime::now();
	
	bool got_valid_grasp_pose;
	bool got_valid_pre_grasp_pose;
	bool got_valid_optimal_grasp_pose;
	
	got_valid_grasp_pose=0;
	got_valid_pre_grasp_pose=0;
	got_valid_optimal_grasp_pose=0;
	
	bool reset_next_grasp_pointer=req.new_situation.data;
	
	while (!(got_valid_grasp_pose&&got_valid_pre_grasp_pose&&got_valid_optimal_grasp_pose))
	{
		cob_grasp_generator::COBNextTestGrasp::Response COBNextTestGrasp_res;
		COBNextTestGrasp_res= GetNextTestGrasp(req, reset_next_grasp_pointer);
		if (!COBNextTestGrasp_res.grasp.grasp_available.data)
		{
			ROS_INFO("ALL GRASP CHECKED ::No Valid Grasp");
			res.success.data=false;
			ros::WallTime end = ros::WallTime::now();
			ros::WallDuration dur = end - start;
			ROS_INFO("cob_grasp_selector took %f seconds", dur.toSec());
			return true;	
		}
		
		else
		{
			geometry_msgs::Pose valid_pre_grasp_pose;	
			valid_pre_grasp_pose= PoseFromObjectToSDHPalmLink(COBNextTestGrasp_res.grasp.pre_grasp_pose, req);
			
			sensor_msgs::JointState valid_pre_grasp_hand_configuration;		
			valid_pre_grasp_hand_configuration=MapHandConfiguration(COBNextTestGrasp_res.grasp.hand_pre_grasp_config);
			
			cob_grasp_validity_checker::COBGetTestGraspValidity::Request COBGetTestGraspValidity_req;
			COBGetTestGraspValidity_req.pose=valid_pre_grasp_pose;  
			COBGetTestGraspValidity_req.joint_position=valid_pre_grasp_hand_configuration;
			
			cob_grasp_validity_checker::COBGetTestGraspValidity::Response COBGetTestGraspValidity_res;
			
			COBGetTestGraspValidity_res= CheckTestGraspValidity(COBGetTestGraspValidity_req);
			got_valid_pre_grasp_pose=COBGetTestGraspValidity_res.success.data;
						
			if(got_valid_pre_grasp_pose)
			{
				geometry_msgs::Pose valid_grasp_pose;	
				valid_grasp_pose= PoseFromObjectToSDHPalmLink(COBNextTestGrasp_res.grasp.grasp_pose, req);
			
				sensor_msgs::JointState valid_grasp_hand_configuration;		
				valid_grasp_hand_configuration=MapHandConfiguration(COBNextTestGrasp_res.grasp.hand_grasp_config);
			
				COBGetTestGraspValidity_req.pose=valid_grasp_pose; 
				COBGetTestGraspValidity_req.joint_position=valid_grasp_hand_configuration;
			
				COBGetTestGraspValidity_res= CheckTestGraspValidity(COBGetTestGraspValidity_req);
				got_valid_grasp_pose=COBGetTestGraspValidity_res.success.data;

				if(got_valid_grasp_pose)
				{	
					geometry_msgs::Pose valid_optimal_grasp_pose;	
					valid_optimal_grasp_pose= PoseFromObjectToSDHPalmLink(COBNextTestGrasp_res.grasp.grasp_pose, req);
			
					sensor_msgs::JointState valid_optimal_grasp_hand_configuration;		
					valid_optimal_grasp_hand_configuration=MapHandConfiguration(COBNextTestGrasp_res.grasp.hand_optimal_grasp_config);

					COBGetTestGraspValidity_req.pose=valid_optimal_grasp_pose;  
					COBGetTestGraspValidity_req.joint_position=valid_optimal_grasp_hand_configuration;
			
					COBGetTestGraspValidity_res= CheckTestGraspValidity(COBGetTestGraspValidity_req);
					got_valid_optimal_grasp_pose=COBGetTestGraspValidity_res.success.data;
					
					if(got_valid_optimal_grasp_pose)
					{
						ROS_INFO("Got Valid Grasp");
						res.success.data=true;
						res.valid_grap_pose=valid_grasp_pose;
						res.valid_pre_grap_pose=valid_pre_grasp_pose;
						res.valid_pre_grasp_joint_position=valid_pre_grasp_hand_configuration;
						res.valid_grasp_joint_position=valid_grasp_hand_configuration;
						res.valid_optimal_grasp_joint_position=valid_optimal_grasp_hand_configuration;
						res.valid_grasp_index=COBNextTestGrasp_res.grasp.index;
						return true;
					}
				}
			}
		}
		reset_next_grasp_pointer=false;
	}
}	
	

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_grasp_selector");

	COBGraspSelector*  cob_grasp_selector= new COBGraspSelector();
	cob_grasp_selector->Init();
	cob_grasp_selector->Run();

	return 0;
}



