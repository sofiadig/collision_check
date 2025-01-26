#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// bool stateFeasibilityTestExample(const moveit::core::RobotState& kinematic_state, bool /*verbose*/)
// {
//   const double* joint_values = kinematic_state.getJointPositions("panda_joint1");
//   return (joint_values[0] > 0.0);
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "two_objects_basic");
  std::cout << "Let's initialize a scene with one panda robot and two collision objects.";
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;

  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  // planning_scene::PlanningScene planning_scene(kinematic_model);

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // ---------------------------------
  moveit_msgs::AttachedCollisionObject box;
  box.link_name = "panda_hand";
  box.object.header.frame_id = "panda_hand";
  box.object.id = "box";

  geometry_msgs::Pose pose;
  pose.position.z = 0.11;
  pose.orientation.w = 1.0;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.075;
  primitive.dimensions[1] = 0.075;
  primitive.dimensions[2] = 0.075;

  box.object.primitives.push_back(primitive);
  box.object.primitive_poses.push_back(pose);
  box.object.operation = box.object.ADD;

  ROS_INFO("Adding the object into the world at the location of the hand.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(box.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


  ros::shutdown();
  return 0;
}
