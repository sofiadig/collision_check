#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "interactivity/interactive_robot.h"
#include "interactivity/pose_string.h"
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
shapes::ShapePtr g_world_cube_shape;
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
  Eigen::Isometry3d world_cube_pose;
  double world_cube_size;
  robot.getWorldGeometry(world_cube_pose, world_cube_size);
  g_planning_scene->getWorldNonConst()->moveShapeInObject("world_cube", g_world_cube_shape, world_cube_pose);

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
  //ros::AsyncSpinner spinner(1);
  //spinner.start();
  ros::NodeHandle node_handle;

  InteractiveRobot robot;
  g_planning_scene = new planning_scene::PlanningScene(robot.robotModel());

//################################################################################

  Eigen::Isometry3d cylinder_pose = Eigen::Isometry3d(Eigen::Translation3d(0.25, -0.5, 0.5));
  //cylinder_pose.translation() = Eigen::Vector3d(0.0, -0.5, 0.0);

  // Define the shape using shapes::Cylinder
  double cylinder_height = 0.4;
  double cylinder_radius = 0.03;
  shapes::ShapePtr cylinder_shape(new shapes::Cylinder(cylinder_radius, cylinder_height));

  // Add the cylinder to the planning scene
  

  Eigen::Isometry3d world_cube_pose;
  double world_cube_size;
  robot.getWorldGeometry(world_cube_pose, world_cube_size);
  g_world_cube_shape.reset(new shapes::Box(world_cube_size, world_cube_size, world_cube_size));
  g_planning_scene->getWorldNonConst()->addToObject("world_cube", g_world_cube_shape, world_cube_pose);
  g_planning_scene->getWorldNonConst()->addToObject("cylinder", cylinder_shape, cylinder_pose);

  ROS_INFO("Starting collision detection.");

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
