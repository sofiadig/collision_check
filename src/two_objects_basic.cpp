#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "interactivity/interactive_robot.h"
#include "interactivity/pose_string.h"
//src/moveit_tutorials/doc/interactivity/include/interactivity/pose_string.h
// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

planning_scene::PlanningScene* g_planning_scene = nullptr;
ros::Publisher* g_marker_array_publisher = nullptr;
visualization_msgs::MarkerArray g_collision_points;

void publishMarkers(visualization_msgs::MarkerArray& markers)
{
  // delete old markers
  if (!g_collision_points.markers.empty())
  {
    for (auto& marker : g_collision_points.markers)
      marker.action = visualization_msgs::Marker::DELETE;

    g_marker_array_publisher->publish(g_collision_points);
  }

  // move new markers into g_collision_points
  std::swap(g_collision_points.markers, markers.markers);

  // draw new markers (if there are any)
  if (!g_collision_points.markers.empty())
    g_marker_array_publisher->publish(g_collision_points);
}

void computeCollisionContactPoints(InteractiveRobot& robot)
{
  g_planning_scene->getWorldNonConst();

  // Collision Requests
  collision_detection::CollisionRequest c_req;
  collision_detection::CollisionResult c_res;
  c_req.group_name = robot.getGroupName();
  c_req.contacts = true;
  c_req.max_contacts = 100;
  c_req.max_contacts_per_pair = 5;
  c_req.verbose = false;

  // Checking for Collisions
  g_planning_scene->checkCollision(c_req, c_res, *robot.robotState());

  if (c_res.collision)
  {
    ROS_INFO_STREAM("COLLIDING contact_point_count: " << c_res.contact_count);
    if (c_res.contact_count > 0)
    {
      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 0.5;
      visualization_msgs::MarkerArray markers;

      /* Get the contact points and display them as markers */
      collision_detection::getCollisionMarkersFromContacts(markers, "panda_link0", c_res.contacts, color,
                                                           ros::Duration(),  // remain until deleted
                                                           0.01);            // radius
      publishMarkers(markers);
    }
  }

  else
  {
    ROS_INFO("Not colliding");

    // delete the old collision point markers
    visualization_msgs::MarkerArray empty_marker_array;
    publishMarkers(empty_marker_array);
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "two_objects_basic");
  std::cout << "Let's initialize a scene with one panda robot and two collision objects.";
  ros::AsyncSpinner spinner(1);
  //spinner.start();

  ros::NodeHandle node_handle;

  InteractiveRobot robot;
  g_planning_scene = new planning_scene::PlanningScene(robot.robotModel());

  // moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  // visual_tools.deleteAllMarkers();

  // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  // planning_scene::PlanningScene planning_scene(kinematic_model);

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // --------------Add first object: Box -------------------
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

  moveit_msgs::ObjectColor color;
  color.id = "box";
  color.color.r = 0.0f; // Blue
  color.color.g = 0.0f;
  color.color.b = 1.0f;
  color.color.a = 0.5f; // 50% transparent
  

  ROS_INFO("Adding the first object into the world at the location of the hand.");
  moveit_msgs::PlanningScene m_planning_scene;
  m_planning_scene.world.collision_objects.push_back(box.object);
  m_planning_scene.object_colors.push_back(color);
  m_planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(m_planning_scene);
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // --------------Add second object: Cylinder -------------------

  moveit_msgs::AttachedCollisionObject cylinder;
  cylinder.link_name = "panda_hand";
  cylinder.object.header.frame_id = "panda_link0";
  cylinder.object.id = "cylinder";

  geometry_msgs::Pose pose2;
  pose2.position.x = 0.3;
  pose2.position.y = -0.5;
  pose2.position.z = 0.5;
  pose2.orientation.w = 1.0;

  shape_msgs::SolidPrimitive primitive2;
  primitive2.type = primitive2.CYLINDER;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = 0.4;
  primitive2.dimensions[1] = 0.03;

  cylinder.object.primitives.push_back(primitive2);
  cylinder.object.primitive_poses.push_back(pose2);
  cylinder.object.operation = cylinder.object.ADD;

  moveit_msgs::ObjectColor color2;
  color2.id = "cylinder";
  color2.color.r = 0.0f; // Blue
  color2.color.g = 1.0f;
  color2.color.b = 0.0f;
  color2.color.a = 0.5f;

  ROS_INFO("Adding the second object into the world at the location of the hand.");
  //moveit_msgs::PlanningScene m_planning_scene;
  m_planning_scene.world.collision_objects.push_back(cylinder.object);
  m_planning_scene.object_colors.push_back(color2);
  m_planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(m_planning_scene);
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//################################################################################

  ROS_INFO("Starting collision detection.");
  //InteractiveRobot robot;
  //g_planning_scene->usePlanningSceneMsg(m_planning_scene);

  // Create a marker array publisher for publishing contact points
  g_marker_array_publisher =
      new ros::Publisher(node_handle.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 100));

  robot.setUserCallback(computeCollisionContactPoints);

  ros::spin();
  delete g_planning_scene;
  delete g_marker_array_publisher;
  ros::shutdown();
  return 0;
}
