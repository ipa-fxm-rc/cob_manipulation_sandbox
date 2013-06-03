#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <planning_environment/models/collision_models.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <cob_grasp_validity_checker/COBGetTestGraspValidity.h>
#include <iostream>
#include <fstream>
#include <cob_grasp_validity_checker/cob_grasp_validity_checker.h>
#include <cstdlib>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "get_grasp_validity_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<cob_grasp_validity_checker::COBGetTestGraspValidity>("grasp_validity_server");
     ROS_INFO("step 1");
cob_grasp_validity_checker::COBGetTestGraspValidity srv;
srv.request.pose.position.x= 2.0;
srv.request.pose.position.y= 2.0;
srv.request.pose.position.z= 0.50;
srv.request.pose.orientation=tf::createQuaternionMsgFromYaw(0.0);
 ROS_INFO("step 2");
//std::vector<float> srv.request.joint_position.position;
//srv.request.joint_position.position={1.047,-0.262,1.047,-0.262,1.047,-0.262,1.047};
srv.request.joint_position.position.push_back(1.047);
srv.request.joint_position.position.push_back(-0.262);
srv.request.joint_position.position.push_back(1.047);
srv.request.joint_position.position.push_back(-0.262);
srv.request.joint_position.position.push_back(1.047);
srv.request.joint_position.position.push_back(-0.262);
srv.request.joint_position.position.push_back(1.047);
 ROS_INFO("step 3");

if (client.call(srv))
{
	ROS_INFO("step 4");
  if (srv.response.success.data)
  {
	  ROS_INFO("step 5");
    ROS_INFO("valid configuration");
  }
  else
  {
	  	  ROS_INFO("step 6");
    ROS_INFO("invalid configuration");
    return 1;
  }
}  
else
{
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
}

  return 0;
}
