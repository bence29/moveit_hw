#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_example/utils.hpp"

void SetupScene()
{
  const auto [object, color] = []
  {
    moveit_msgs::msg::CollisionObject object;
    // Place plane
    object.id = "ground";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions.resize(3);
    object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 10;
    object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 10;
    object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.01;
    object.primitive_poses.resize(1);
    object.primitive_poses[0].position.x = 0.0;
    object.primitive_poses[0].position.y = 0.0;
    object.primitive_poses[0].position.z = -0.01;
    object.primitive_poses[0].orientation.w = 1.0;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Specify the plane's color
    std_msgs::msg::ColorRGBA color;
    color.r = 0.7;
    color.g = 0.3;
    color.b = 0.2;
    color.a = 1.0;

    return std::make_pair(object, color);
  }();

  // Add the ground to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(object, color);
}
