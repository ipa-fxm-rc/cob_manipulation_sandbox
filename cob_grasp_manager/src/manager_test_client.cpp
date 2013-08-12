#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cob_grasp_manager/cob_grasp_manager.h>



int main(int argc, char **argv)
{

  ros::init(argc, argv, "grasp_manager_client");
    ros::NodeHandle gm;
    ros::ServiceClient client = gm.serviceClient<cob_grasp_manager::COBGetObjectGrasp>("object_grasp_provider");
     ROS_INFO("step 1");
cob_grasp_manager::COBGetObjectGrasp srv;

srv.request.object_name="salt";
srv.request.objectID=13;
srv.request.new_situation.data=true;


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
