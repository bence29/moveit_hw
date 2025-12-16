#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_hw/utils.hpp"

void SetupScene()
{
  const auto [objects, colors] = []
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects(3);

    // Place first table
    objects[0].id = "table1";
    objects[0].header.frame_id = "world";
    objects[0].primitives.resize(1);
    objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    objects[0].primitives[0].dimensions.resize(3);
    objects[0].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.4;
    objects[0].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.4;
    objects[0].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.4;
    objects[0].primitive_poses.resize(1);
    objects[0].primitive_poses[0].position.x = 0.6;
    objects[0].primitive_poses[0].position.y = -0.25;
    objects[0].primitive_poses[0].position.z = 0.2;
    objects[0].primitive_poses[0].orientation.w = 1.0;
    objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Place second table
    objects[1].id = "table2";
    objects[1].header.frame_id = "world";
    objects[1].primitives.resize(1);
    objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    objects[1].primitives[0].dimensions.resize(3);
    objects[1].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.4;
    objects[1].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.4;
    objects[1].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.4;
    objects[1].primitive_poses.resize(1);
    objects[1].primitive_poses[0].position.x = 0.6;
    objects[1].primitive_poses[0].position.y = 0.25;
    objects[1].primitive_poses[0].position.z = 0.2;
    objects[1].primitive_poses[0].orientation.w = 1.0;
    objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Place cylinder on the first table
    objects[2].id = "cylinder";
    objects[2].header.frame_id = "world";
    objects[2].primitives.resize(1);
    objects[2].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    objects[2].primitives[0].dimensions.resize(2);
    objects[2].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.1;
    objects[2].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.02;
    objects[2].primitive_poses.resize(1);
    objects[2].primitive_poses[0].position.x = 0.6;
    objects[2].primitive_poses[0].position.y = 0.25;
    objects[2].primitive_poses[0].position.z = 0.45 + 1e-3;
    objects[2].primitive_poses[0].orientation.w = 1.0;
    objects[2].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Specify the color of each object in the scene
    std::vector<moveit_msgs::msg::ObjectColor> colors(3);

    // Specify the first table's color
    colors[0].color.r = 1.0;
    colors[0].color.g = 0.0;
    colors[0].color.b = 0.0;
    colors[0].color.a = 1.0;

    // Specify the second table's color
    colors[1].color.r = 0.0;
    colors[1].color.g = 0.0;
    colors[1].color.b = 1.0;
    colors[1].color.a = 1.0;

    // Specify the cone's color
    colors[2].color.r = 0.5;
    colors[2].color.g = 0.0;
    colors[2].color.b = 0.5;
    colors[2].color.a = 1.0;

    return std::make_pair(objects, colors);
  }();

  // Add the collision objects to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(objects, colors);
}

void SecondScenario(  //ez a "main"
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    const rclcpp::Logger &logger)
{
  moveit_visual_tools.prompt("Press 'Next' to start the second scenario");

  SetupScene();
  
  moveit_visual_tools.prompt("Press 'Next' to plan to the cylinder");

  // Find the position of the cylinder

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const auto cylinder_pos = GetObjectPosition("cylinder");
  RCLCPP_INFO(logger, "Cylinder position: %f %f %f", cylinder_pos.x, cylinder_pos.y, cylinder_pos.z);

  // Move the end-effector above the cylinder
  const auto pick_up_pose = [&cylinder_pos]
  {
    geometry_msgs::msg::Pose msg;

    msg.orientation.x = 0.0;
    msg.orientation.y = std::sin(Constants::PI / 2);
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 2);

    msg.position.x = cylinder_pos.x;
    msg.position.y = cylinder_pos.y;
    msg.position.z = cylinder_pos.z + 0.06; //+ 0.06;
    return msg;
  }();
  MoveToPose(pick_up_pose, move_group_interface, moveit_visual_tools, logger);

  // Attach the cylinder to the end-effector
  move_group_interface.attachObject("cylinder");
  moveit_visual_tools.prompt("Cylinder attached to the end-effector. Press 'Next' to continue");

  // Make sure that the orientation doesn't change
  const auto constraints = []
  {
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "tool0";
    ocm.header.frame_id = "world";

    ocm.orientation.x = 0.0;
    ocm.orientation.y = std::sin(Constants::PI / 2);
    ocm.orientation.z = 0.0;
    ocm.orientation.w = std::cos(Constants::PI / 2);

    // Specify max deviation in radians
    ocm.absolute_x_axis_tolerance = 0.5;
    ocm.absolute_y_axis_tolerance = 0.5;
    ocm.absolute_z_axis_tolerance = 0.5;
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    return constraints;
  }();
  move_group_interface.setPathConstraints(constraints);

  // Find position of the first table
  const auto drop_off_position = GetObjectPosition("table1");
  RCLCPP_INFO(logger, "Target table position: %f %f %f", drop_off_position.x, drop_off_position.y, drop_off_position.z);

  // Move the end-effector to the drop off point
  const auto drop_off_pose = [&drop_off_position]
  {
    geometry_msgs::msg::Pose msg;

    msg.orientation.x = 0.0;
    msg.orientation.y = std::sin(Constants::PI / 2);
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 2);

    msg.position.x = drop_off_position.x;
    msg.position.y = drop_off_position.y;
    msg.position.z = drop_off_position.z + 0.2 + 0.11 + 1e-3;
    return msg;
  }();
  move_group_interface.setPlanningTime(15.0);
  MoveToPose(drop_off_pose, move_group_interface, moveit_visual_tools, logger);

  // Detach the cylinder from the end-effector
  move_group_interface.detachObject("cylinder");
  moveit_visual_tools.prompt("Cylinder detached from the end-effector. Press 'Next' to continue");

  // Clear path constraints
  move_group_interface.clearPathConstraints();

  // Move to the home position
  MoveToHome(move_group_interface, moveit_visual_tools, logger);
}
