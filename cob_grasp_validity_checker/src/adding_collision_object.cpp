#include <ros/ros.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/collision_models.h>

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

int main(int argc, char **argv){
  ros::init (argc, argv, "get_state_validity_test");
  ros::NodeHandle rh;

  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;

  vis_marker_publisher_ = rh.advertise<visualization_msgs::Marker>("state_validity_markers", 128);
  vis_marker_array_publisher_ = rh.advertise<visualization_msgs::MarkerArray>("state_validity_markers_array", 128);

  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  ros::ServiceClient get_planning_scene_client = 
    rh.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

  arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
  arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

  if(argc > 1) {

    std::stringstream s(argv[1]);
    bool add;
    s >> add;

    if(add) {
      //add the cylinder into the collision space
      arm_navigation_msgs::CollisionObject cylinder_object;
      cylinder_object.id = "pole";
      cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
      cylinder_object.header.frame_id = "odom_combined";
      cylinder_object.header.stamp = ros::Time::now();
      arm_navigation_msgs::Shape object;
      object.type = arm_navigation_msgs::Shape::CYLINDER;
      object.dimensions.resize(2);
      object.dimensions[0] = .1;
      object.dimensions[1] = 1.0;
      geometry_msgs::Pose pose;
      pose.position.x = .6;
      pose.position.y = -.6;
      pose.position.z = .5;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
      cylinder_object.shapes.push_back(object);
      cylinder_object.poses.push_back(pose);
      
      planning_scene_req.planning_scene_diff.collision_objects.push_back(cylinder_object);
    }
  }

  if(!get_planning_scene_client.call(planning_scene_req, planning_scene_res)) {
    ROS_WARN("Can't get planning scene");
    return -1;
  }

  planning_environment::CollisionModels collision_models("robot_description");
  planning_models::KinematicState* state = 
    collision_models.setPlanningScene(planning_scene_res.planning_scene);
