#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_hw/utils.hpp"
#include "moveit_hw/second_scenario.hpp"

#include <string>

void SetupScene2(const rclcpp::Node::SharedPtr &node) //A node-ot végig át kellett adni a függvényeken, hogy kiolvasható legyen a yaml-ből a paraméter
{
  const auto [object, color] = [node]() 
  {                                     //Itt is át kell adni a lambda függvénynek
    std::vector<moveit_msgs::msg::CollisionObject> object(17);
    object[0].id = "euro_pallet";
    object[0].header.frame_id = "world";

    //Global Offszetek
    const double X_global_offs = node->get_parameter("X_global_offs_p").as_double();
    const double Y_global_offs = node->get_parameter("Y_global_offs_p").as_double();
    const double Z_global_offs = node->get_parameter("Z_global_offs_p").as_double();

    // A teljes raklap méretei (m)
    const double L = 1.20;   // hossz
    const double W = 0.80;   // szélesség
    const double H = 0.145;  // magasság

    // Deszka és kocka méretek
    const double plank_thickness = 0.022;  // 22 mm
    const double top_plank_width = 0.145;  // felső deszkák szélessége
    const double bottom_plank_width = 0.145;
    const double middle_plank_width = 0.145;
    const double middle_plank_height = 0.101; //köztes deszkák magassága

    // Hány primitívet használunk?
    object[0].primitives.resize(11);
    object[0].primitive_poses.resize(11);

    // Index segéd
    int i = 0;

    // 1) FELSŐ 5 DESZKA
    for (int p = 0; p < 5; p++)
    {
      object[0].primitives[i].type = shape_msgs::msg::SolidPrimitive::BOX;
      object[0].primitives[i].dimensions = { L, top_plank_width, plank_thickness };

      double y_offset = -W/2 + top_plank_width/2 + p * (top_plank_width + 0.01875); //0.01875 kiszámolt pontos érték

      object[0].primitive_poses[i].position.x = X_global_offs;
      object[0].primitive_poses[i].position.y = Y_global_offs + y_offset;
      object[0].primitive_poses[i].position.z = Z_global_offs + H - plank_thickness/2;

      object[0].primitive_poses[i].orientation.w = 1.0;
      i++;
    }

    // 2) KÖZÉPSŐ 3 DESZKA
    for (int b = 0; b < 3; b++)
    {
      object[0].primitives[i].type = shape_msgs::msg::SolidPrimitive::BOX;
      object[0].primitives[i].dimensions = { middle_plank_width, W, middle_plank_height };
      
      double x_offset = -L/2 + middle_plank_width/2 + b * (L/2 - middle_plank_width/2);
      object[0].primitive_poses[i].position.x = X_global_offs + x_offset;
      object[0].primitive_poses[i].position.y = Y_global_offs;
      object[0].primitive_poses[i].position.z = Z_global_offs + plank_thickness + middle_plank_height/2;

      object[0].primitive_poses[i].orientation.w = 1.0;
      i++;
    }

    // 3) ALSÓ 3 DESZKA
    for (int p = 0; p < 3; p++)
    {
      object[0].primitives[i].type = shape_msgs::msg::SolidPrimitive::BOX;
      object[0].primitives[i].dimensions = { L, bottom_plank_width, plank_thickness };

      double y_offset = -W/2 + bottom_plank_width/2 + p * (W/2 - bottom_plank_width/2);

      object[0].primitive_poses[i].position.x = X_global_offs;
      object[0].primitive_poses[i].position.y = Y_global_offs + y_offset;
      object[0].primitive_poses[i].position.z = Z_global_offs + plank_thickness/2;

      object[0].primitive_poses[i].orientation.w = 1.0;
      i++;
    }
    // Objektum hozzáadása
    object[0].operation = moveit_msgs::msg::CollisionObject::ADD;  //Egy objektum van több primitívvel/geometriával, és szintén 1 színnel.

    // Define the color of the box
    // Specify the color of each object in the scene
    std::vector<moveit_msgs::msg::ObjectColor> color(17);
    color[0].color.r = 0.796;
    color[0].color.g = 0.612;
    color[0].color.b = 0.384;
    color[0].color.a = 1.0;

    // 16 kezdő objektum a raklapra, 4x4-esen, 1-essel kezdődik a raklap miatt
    double placementWidth = 0.14;
    double placementWidthGap = 0.03;
    double placementLength = 0.18;
    double placementLengthGap = 0.06;
    double placementHeight = 0.015;
    for(int j = 1; j < 17; j++) {
      double x_pl_offs = - L/2 + placementLengthGap + placementLength/2 + ((j-1)%4) * (2*placementLengthGap+placementLength);
      //y offszetek állítása:
      double y_pl_offs = 0;
      if(j < 5)
        y_pl_offs = - W/2 + placementWidthGap + placementWidth/2;
      else if(j < 9)
        y_pl_offs = - W/2 + placementWidthGap + placementWidth/2 + (2*placementWidthGap + placementWidth);
      else if(j < 13)
        y_pl_offs = - W/2 + placementWidthGap + placementWidth/2 + 2*(2*placementWidthGap + placementWidth);
      else
        y_pl_offs = - W/2 + placementWidthGap + placementWidth/2 + 3*(2*placementWidthGap + placementWidth);
      
      // std::string unique_id;
      object[j].id = ("startingPoint_" + std::to_string(j));
      object[j].header.frame_id = "world";
      object[j].primitives.resize(1);
      object[j].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
      object[j].primitives[0].dimensions = { placementLength, placementWidth, placementHeight };
      object[j].primitive_poses.resize(1);
      object[j].primitive_poses[0].position.x = X_global_offs + x_pl_offs;
      object[j].primitive_poses[0].position.y = Y_global_offs + y_pl_offs;
      object[j].primitive_poses[0].position.z = Z_global_offs + H + placementHeight/2;
      object[j].primitive_poses[0].orientation.w = 1.0;
      object[j].operation = moveit_msgs::msg::CollisionObject::ADD;

      // A 16 kezdő objektum színe
      color[j].color.r = 0.0;
      color[j].color.g = 1.0;
      color[j].color.b = 0.0;
      color[j].color.a = 0.2;
    }

    return std::make_pair(object, color);
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(object, color);  //Object vagy Objects attól függően, hogy egy vagy többről van szó!
}

void ThirdScenario(  //ez a "main"
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
  const rclcpp::Logger &logger, const rclcpp::Node::SharedPtr &node)
{
  moveit_visual_tools.prompt("Press 'Next' to start the third scenario");

  SetupScene2(node);

  // moveit_visual_tools.prompt("Press 'Next' to plan to the centerPoint");

  // Find the position of the startingPoint_1
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const auto centerPoint_pos = GetObjectPosition("startingPoint_1");
  RCLCPP_INFO(logger, "startingPoint_1 position: %f %f %f", centerPoint_pos.x, centerPoint_pos.y, centerPoint_pos.z);

  // Move the end-effector above the startingPoint_1
  auto pick_up_pose = [&centerPoint_pos]
  {
    geometry_msgs::msg::Pose msg;

    msg.orientation.x = 0.0;
    msg.orientation.y = std::sin(Constants::PI / 2);
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 2);

    msg.position.x = centerPoint_pos.x;
    msg.position.y = centerPoint_pos.y;
    msg.position.z = centerPoint_pos.z + 0.06; //+ 0.06;
    return msg;
  }();

  moveit_visual_tools.prompt("Press 'Next' to move to the centerPoint");
  MoveToPose(pick_up_pose, move_group_interface, moveit_visual_tools, logger);

  // Attach the cylinder to the end-effector
  move_group_interface.attachObject("startingPoint_1");
  moveit_visual_tools.prompt("startingPoint_1 attached to the end-effector. Press 'Next' to continue");

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

  //Going above startingPoint_1
  pick_up_pose.position.z += 0.1;
  move_group_interface.setPlanningTime(20.0);
  MoveToPose(pick_up_pose, move_group_interface, moveit_visual_tools, logger);

  // Find the position of the startingPoint_2
  const auto startingPoint_2_pos = GetObjectPosition("startingPoint_2");
  RCLCPP_INFO(logger, "startingPoint_2 position: %f %f %f", startingPoint_2_pos.x, startingPoint_2_pos.y, startingPoint_2_pos.z);

  // Move the end-effector above the startingPoint_2
  auto pick_up_pose2 = [&startingPoint_2_pos]
  {
    geometry_msgs::msg::Pose msg;

    msg.orientation.x = 0.0;
    msg.orientation.y = std::sin(Constants::PI / 2);
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 2);

    msg.position.x = startingPoint_2_pos.x;
    msg.position.y = startingPoint_2_pos.y;
    msg.position.z = startingPoint_2_pos.z + 0.115 + 0.06; //+ 0.06;
    return msg;
  }();
  moveit_visual_tools.prompt("Press 'Next' to move to the startingPoint_2");
  move_group_interface.setPlanningTime(20.0);
  MoveToPose(pick_up_pose2, move_group_interface, moveit_visual_tools, logger);

  pick_up_pose2.position.z -= 0.1;
  move_group_interface.setPlanningTime(20.0);
  MoveToPose(pick_up_pose2, move_group_interface, moveit_visual_tools, logger);

  // Detach the cylinder from the end-effector
  move_group_interface.detachObject("startingPoint_1");
  moveit_visual_tools.prompt("startingPoint_1 detached from the end-effector. Press 'Next' to continue");

  // Clear path constraints
  move_group_interface.clearPathConstraints();

  // Move to the home position
  moveit_visual_tools.prompt("Press 'Next' to move to the home position");
  MoveToHome(move_group_interface, moveit_visual_tools, logger);
}


  moveit_visual_tools.prompt("Press 'Next' to move to the centerPoint");
  for(int tries = 0; tries < 3; tries++) {
    move_group_interface.setPlanningTime(30.0);
    bool pickupbool = MoveToPose(box_pick_up_pose, move_group_interface, moveit_visual_tools, logger);
    if(pickupbool)
      break;
    else if(tries == 2)
      exit;
  }