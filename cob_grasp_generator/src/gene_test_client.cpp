#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <planning_environment/models/collision_models.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cob_grasp_generator/COBNextTestGrasp.h>
int main(int argc, char **argv)
{

  ros::init(argc, argv, "generate_grasp");
    ros::NodeHandle gm;
    ros::ServiceClient client = gm.serviceClient<cob_grasp_generator::COBNextTestGrasp>("get_next_grasp_server");

cob_grasp_generator::COBNextTestGrasp srv;
//srv.request.pose.position.x= 0.0;
//srv.request.pose.position.y= 0.0;
//srv.request.pose.position.z= 0.50;
//srv.request.pose.orientation=tf::createQuaternionMsgFromYaw(0.0);
 //ROS_INFO("step 2");
////std::vector<float> srv.request.joint_position.position;
////srv.request.joint_position.position={1.047,-0.262,1.047,-0.262,1.047,-0.262,1.047};
//srv.request.joint_position.position.push_back(1.047);
//srv.request.joint_position.position.push_back(-0.262);
//srv.request.joint_position.position.push_back(1.047);
//srv.request.joint_position.position.push_back(-0.262);
//srv.request.joint_position.position.push_back(1.047);
//srv.request.joint_position.position.push_back(-0.262);
//srv.request.joint_position.position.push_back(1.047);
 //ROS_INFO("step 3");

srv.request.objectID=11;
ROS_INFO("step clnt 1");
ros::service::waitForService("get_next_grasp_server");
for(int i=0; i<6;i++)
{
if (client.call(srv))
{
	ROS_INFO("gene_service_working");
	for (int im=0;im<srv.response.grasp.hand_grasp_config.position.size(); im++)
{
	ROS_INFO("%f",srv.response.grasp.hand_grasp_config.position[im]);
}
}

else
{
ROS_INFO("gene_service_not_working");
}
}
/*  if (srv.response.success.data)
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
*/
  return 0;
}
