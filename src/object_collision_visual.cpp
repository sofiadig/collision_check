/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Acorn Pooley, Michael Lautman */

// This code goes with the Collision Contact Visualization tutorial

#include <ros/ros.h>
#include "/home/sdg/ws_moveit/src/moveit_tutorials/doc/interactivity/include/interactivity/interactive_robot.h"
// Path for interactive_robot.h: /home/sdg/ws_moveit/src/moveit_tutorials/doc/interactivity/include/interactivity/interactive_robot.h
#include "/home/sdg/ws_moveit/src/moveit_tutorials/doc/interactivity/include/interactivity/pose_string.h"

// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

planning_scene::PlanningScene* g_planning_scene = nullptr;
shapes::ShapePtr g_world_cube_shape;
shapes::ShapePtr g_world_cylinder_shape;
ros::Publisher* g_marker_array_publisher = nullptr;
visualization_msgs::MarkerArray g_collision_points;

void help()
{
  ROS_INFO("#####################################################");
  ROS_INFO("RVIZ SETUP");
  ROS_INFO("----------");
  ROS_INFO("  Global options:");
  ROS_INFO("    FixedFrame = /panda_link0");
  ROS_INFO("  Add a RobotState display:");
  ROS_INFO("    RobotDescription = robot_description");
  ROS_INFO("    RobotStateTopic  = interactive_robot_state");
  ROS_INFO("  Add a Marker display:");
  ROS_INFO("    MarkerTopic = interactive_robot_markers");
  ROS_INFO("  Add an InteractiveMarker display:");
  ROS_INFO("    UpdateTopic = interactive_robot_imarkers/update");
  ROS_INFO("  Add a MarkerArray display:");
  ROS_INFO("    MarkerTopic = interactive_robot_marray");
  ROS_INFO("#####################################################");
}

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
  // move the world geometry in the collision world
  Eigen::Isometry3d world_cube_pose;
  Eigen::Isometry3d cylinder_pose;
  double world_cube_size;
  robot.getWorldGeometry(world_cube_pose, world_cube_size);
  g_planning_scene->getWorldNonConst()->moveShapeInObject("world_cube", g_world_cube_shape, world_cube_pose);
  g_planning_scene->getWorldNonConst()->moveShapeInObject("world_cylinder", g_world_cylinder_shape, cylinder_pose);

  // BEGIN_SUB_TUTORIAL computeCollisionContactPoints
  //
  // Collision Requests
  // ^^^^^^^^^^^^^^^^^^
  // We will create a collision request for the Panda robot
  collision_detection::CollisionRequest c_req;
  collision_detection::CollisionResult c_res;
  c_req.group_name = robot.getGroupName();
  c_req.contacts = true;
  c_req.max_contacts = 100;
  c_req.max_contacts_per_pair = 5;
  c_req.verbose = false;

  // Checking for Collisions
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We check for collisions between robot and itself or the world.
  g_planning_scene->checkCollision(c_req, c_res, *robot.robotState());

  // Displaying Collision Contact Points
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // If there are collisions, we get the contact points and display them as markers.
  // **getCollisionMarkersFromContacts()** is a helper function that adds the
  // collision contact points into a MarkerArray message. If you want to use
  // the contact points for something other than displaying them you can
  // iterate through **c_res.contacts** which is a std::map of contact points.
  // Look at the implementation of getCollisionMarkersFromContacts() in
  // `collision_tools.cpp
  // <https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_core/collision_detection/src/collision_tools.cpp>`_
  // for how.
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
  // END_SUB_TUTORIAL
  else
  {
    ROS_INFO("Not colliding");

    // delete the old collision point markers
    visualization_msgs::MarkerArray empty_marker_array;
    publishMarkers(empty_marker_array);
  }
}

moveit_msgs::CollisionObject createObject(const ros::NodeHandle& pnh) {
  std::string object_reference_frame = "panda_link0";
	std::vector<double> object_dimensions = {0.25, 0.02};
	geometry_msgs::Pose pose;// = {0.6, 0.5, 1.1, 0, 0, 0};

	moveit_msgs::CollisionObject object;
	object.id = "world_cylinder";
	object.header.frame_id = "panda_link0";
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions.resize(2);
  object.primitives[0].dimensions[0] = 0.5;   // height
  object.primitives[0].dimensions[1] = 0.1;   // radius
  object.primitives[0].dimensions = object_dimensions;
  pose.position.x = 0.6;
  pose.position.y = 0.5;
  pose.position.z = 1.1;
  pose.orientation.w = 1.0;
  // pose.orientation.x = 0;
  // pose.orientation.y = 0;
  // pose.orientation.z = 0;
	object.primitive_poses.push_back(pose);
  object.operation = object.ADD;
	return object;
}

void setupSceneObject(ros::NodeHandle& pnh) {
	// Add object to planning scene
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	//moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::CollisionObject newCylinder = createObject(pnh);

  // if (!psi.applyCollisionObject(newCylinder))
	// 	throw std::runtime_error("Failed to spawn object: " + newCylinder.id);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualizing_collisions_tutorial");
  ros::NodeHandle nh;

  // BEGIN_TUTORIAL
  //
  // Initializing the Planning Scene and Markers
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // For this tutorial we use an :codedir:`InteractiveRobot <interactivity/src/interactive_robot.cpp>`
  // object as a wrapper that combines a robot_model with the cube and an interactive marker. We also
  // create a :planning_scene:`PlanningScene` for collision checking. If you haven't already gone through the
  // `planning scene tutorial <../planning_scene/planning_scene_tutorial.html>`_, you go through that first.
  

  InteractiveRobot robot;
  /* Create a PlanningScene */
  g_planning_scene = new planning_scene::PlanningScene(robot.robotModel());

  // Adding first geometry (cube) to the PlanningScene
  Eigen::Isometry3d world_cube_pose;
  double world_cube_size;
  robot.getWorldGeometry(world_cube_pose, world_cube_size);
  g_world_cube_shape.reset(new shapes::Box(world_cube_size, world_cube_size, world_cube_size));
  g_planning_scene->getWorldNonConst()->addToObject("world_cube", g_world_cube_shape, world_cube_pose);

  // //setupSceneObject(nh);

  // // Add the cylinder object to the scene message
  // moveit_msgs::CollisionObject cylinder = createObject(nh);
  // planning_scene_msg.world.collision_objects.push_back(cylinder);
  // planning_scene_msg.is_diff = true;

  // // Publish the scene update
  // planning_scene_diff_publisher.publish(planning_scene_msg);
  // ros::Duration(10).sleep();

  // Publish the scene update
  //planning_scene_diff_publisher.publish(g_planning_scene);

  // // Adding second geometry (cylinder) to the PlanningScene
  // Eigen::Isometry3d cylinder_pose;
  // cylinder_pose.setIdentity();
  // // Position the cylinder at x=0.5, y=0.5, z=0.5
  // cylinder_pose.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);
  // // Create a cylinder with radius 0.1 and height 0.3
  // g_world_cylinder_shape.reset(new shapes::Cylinder(0.1, 0.3));
  // g_planning_scene->getWorldNonConst()->addToObject("world_cylinder", g_world_cylinder_shape, cylinder_pose);

  // // After adding cylinder to planning scene
  // visualization_msgs::Marker cylinder_marker;
  // cylinder_marker.header.frame_id = "panda_link0";
  // cylinder_marker.header.stamp = ros::Time::now();
  // cylinder_marker.ns = "world_cylinder";
  // cylinder_marker.id = 1;  // Different ID from cube which is 0
  // cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
  // cylinder_marker.action = visualization_msgs::Marker::ADD;
  // cylinder_marker.pose.position.x = cylinder_pose.translation().x();
  // cylinder_marker.pose.position.y = cylinder_pose.translation().y();
  // cylinder_marker.pose.position.z = cylinder_pose.translation().z();
  // cylinder_marker.pose.orientation.w = 1.0;
  // cylinder_marker.scale.x = 0.1;  // diameter
  // cylinder_marker.scale.y = 0.1;  // diameter
  // cylinder_marker.scale.z = 0.5;  // height
  // cylinder_marker.color.r = 0.0;
  // cylinder_marker.color.g = 1.0;
  // cylinder_marker.color.b = 0.0;
  // cylinder_marker.color.a = 0.7;
  // cylinder_marker.lifetime = ros::Duration(0);  // persistent

  // // Create a marker array and publish
  // visualization_msgs::MarkerArray markers;
  // markers.markers.push_back(cylinder_marker);
  

  // CALL_SUB_TUTORIAL computeCollisionContactPoints
  // END_TUTORIAL

  // Create a marker array publisher for publishing contact points
  g_marker_array_publisher =
      new ros::Publisher(nh.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 100));

  robot.setUserCallback(computeCollisionContactPoints);

  help();

  ros::spin();

  delete g_planning_scene;
  delete g_marker_array_publisher;

  ros::shutdown();
  return 0;
}
