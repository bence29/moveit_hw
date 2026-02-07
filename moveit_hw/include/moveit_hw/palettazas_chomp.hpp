#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_hw/utils.hpp"
#include "moveit_hw/palettazas_ompl.hpp"

#include <string>

void pickAndPlace_chomp(moveit::planning_interface::MoveGroupInterface &move_group_interface,
  moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
  const rclcpp::Logger &logger, const rclcpp::Node::SharedPtr &node, int iterator){
  // Find the position of box_n
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const auto box_pos = GetObjectPosition("box_" + std::to_string(iterator));
  RCLCPP_INFO(logger, "box_%i position: %f %f %f", iterator, box_pos.x, box_pos.y, box_pos.z);

  // Move the end-effector above box_n
  const double Box_height = node->get_parameter("Box_height_p").as_double();
  const auto box_pick_up_pose = [&box_pos, Box_height]
  {
    geometry_msgs::msg::Pose msg;

    msg.orientation.x = 0.0;
    msg.orientation.y = std::sin(Constants::PI / 2);
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 2);

    msg.position.x = box_pos.x;
    msg.position.y = box_pos.y;
    msg.position.z = box_pos.z + Box_height/2 + 0.06; //+ 0.06;
    return msg;
  }();

  // moveit_visual_tools.prompt("Press 'Next' to move to box_" + std::to_string(iterator));
  move_group_interface.setPlanningTime(30.0);
  bool moveSuccess = false;
  for(int i = 0; i < 5; i++){
    if(moveSuccess)
      break;
    else
      moveSuccess = MoveToPose_chomp(box_pick_up_pose, move_group_interface, moveit_visual_tools, logger);
  }
  
  // Attach the cylinder to the end-effector
  move_group_interface.attachObject("box_" + std::to_string(iterator));
  moveit_visual_tools.prompt("box_" + std::to_string(iterator) + " attached to the end-effector. Press 'Next' to continue");

  // Find the position of placement_n
  const auto startingPoint_pos = GetObjectPosition("startingPoint_" + std::to_string(iterator));
  RCLCPP_INFO(logger, "startingPoint_%i position: %f %f %f", iterator, startingPoint_pos.x, startingPoint_pos.y, startingPoint_pos.z);

  // Move the end-effector above placement_n
  const double placementHeight = node->get_parameter("placementHeight_p").as_double();
  const auto put_down_pose = [&startingPoint_pos, Box_height, placementHeight]
  {
    geometry_msgs::msg::Pose msg;

    msg.orientation.x = 0.0;
    msg.orientation.y = std::sin(Constants::PI / 2);
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 2);

    msg.position.x = startingPoint_pos.x;
    msg.position.y = startingPoint_pos.y;
    msg.position.z = startingPoint_pos.z + Box_height + placementHeight + 0.06 + 0.04; //+ 0.06; placementheight fele is kell ÉS a box_1 egész magassága és egy kicsi!
    return msg;
  }();

  // moveit_visual_tools.prompt("Press 'Next' to move to startingPoint_" + std::to_string(iterator));
  move_group_interface.setPlanningTime(30.0);
  moveSuccess = false;
  for(int i = 0; i < 5; i++){
    if(moveSuccess)
      break;
    else
      moveSuccess = MoveToPose_chomp(put_down_pose, move_group_interface, moveit_visual_tools, logger);
  }

  // Detach the box from the end-effector
  move_group_interface.detachObject("box_" + std::to_string(iterator));
  moveit_visual_tools.prompt("box_ " + std::to_string(iterator) + " detached from the end-effector. Press 'Next' to continue");
}

void palettazas_chomp(  //ez a "main"
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
  const rclcpp::Logger &logger, const rclcpp::Node::SharedPtr &node)
{
  moveit_visual_tools.prompt("Press 'Next' to start palettazas");

  SetupScene(node);

  for(int i = 1; i < 17; i++) { //Pick and place és box teremtő loop
  newBox(node, i);
  pickAndPlace_chomp(move_group_interface, moveit_visual_tools, logger, node, i);
  }

  // Clear path constraints
  move_group_interface.clearPathConstraints();

  // Move to the home position
  moveit_visual_tools.prompt("Press 'Next' to move to the home position");
  //MoveToHome(move_group_interface, moveit_visual_tools, logger);
}
