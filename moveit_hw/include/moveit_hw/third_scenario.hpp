#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_hw/utils.hpp"
#include "moveit_hw/second_scenario.hpp"

void SetupScene2()
{
  const auto [object, color] = []
  {
    std::vector<moveit_msgs::msg::CollisionObject> object(2);
    object[0].id = "euro_pallet";
    object[0].header.frame_id = "world";

    //Offszetek
    const double X_offs = 1.6;
    // const double Y_offs = 0.0;
    const double Z_offs = 0.4;

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

      object[0].primitive_poses[i].position.x = X_offs;
      object[0].primitive_poses[i].position.y = y_offset;
      object[0].primitive_poses[i].position.z = Z_offs + H - plank_thickness/2;

      object[0].primitive_poses[i].orientation.w = 1.0;
      i++;
    }

    // 2) KÖZÉPSŐ 3 DESZKA
    for (int b = 0; b < 3; b++)
    {
      object[0].primitives[i].type = shape_msgs::msg::SolidPrimitive::BOX;
      object[0].primitives[i].dimensions = { middle_plank_width, W, middle_plank_height };
      
      double x_offset = -L/2 + middle_plank_width/2 + b * (L/2 - middle_plank_width/2);
      object[0].primitive_poses[i].position.x = X_offs + x_offset;
      object[0].primitive_poses[i].position.y = 0.0;
      object[0].primitive_poses[i].position.z = Z_offs + plank_thickness + middle_plank_height/2;

      object[0].primitive_poses[i].orientation.w = 1.0;
      i++;
    }

    // 3) ALSÓ 3 DESZKA
    for (int p = 0; p < 3; p++)
    {
      object[0].primitives[i].type = shape_msgs::msg::SolidPrimitive::BOX;
      object[0].primitives[i].dimensions = { L, bottom_plank_width, plank_thickness };

      double y_offset = -W/2 + bottom_plank_width/2 + p * (W/2 - bottom_plank_width/2);

      object[0].primitive_poses[i].position.x = X_offs;
      object[0].primitive_poses[i].position.y = y_offset;
      object[0].primitive_poses[i].position.z = Z_offs + plank_thickness/2;

      object[0].primitive_poses[i].orientation.w = 1.0;
      i++;
    }
    // Objektum hozzáadása
    object[0].operation = moveit_msgs::msg::CollisionObject::ADD;  //Egy objektum van több primitívvel/geometriával, és szintén 1 színnel.

    // Define the color of the box
    // Specify the color of each object in the scene
    std::vector<moveit_msgs::msg::ObjectColor> color(2);
    color[0].color.r = 0.796;
    color[0].color.g = 0.612;
    color[0].color.b = 0.384;
    color[0].color.a = 1.0;

    // Egy új objekt a raklap közepére
    object[1].id = "centerPoint";
    object[1].header.frame_id = "world";
    object[1].primitives.resize(1);
    object[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object[1].primitives[0].dimensions = { top_plank_width - 0.01, top_plank_width - 0.01, plank_thickness };
    object[1].primitive_poses.resize(1);
    object[1].primitive_poses[0].position.x = X_offs;
    object[1].primitive_poses[0].position.y = 0.0;
    object[1].primitive_poses[0].position.z = Z_offs + H + plank_thickness/2;
    object[1].primitive_poses[0].orientation.w = 1.0;
    object[1].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Raklap közepének a színe
    color[1].color.r = 0.0;
    color[1].color.g = 1.0;
    color[1].color.b = 0.0;
    color[1].color.a = 0.4;

    return std::make_pair(object, color);
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(object, color);  //Object vagy Objects attól függően, hogy egy vagy többről van szó!
}

void ThirdScenario(  //ez a "main"
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
  const rclcpp::Logger &logger)
{
  moveit_visual_tools.prompt("Press 'Next' to start the third scenario");

  SetupScene2();

  moveit_visual_tools.prompt("Press 'Next' to plan to the ground");

  // Find the position of the ground
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const auto euro_pallet_pos = GetObjectPosition("centerPoint");
  RCLCPP_INFO(logger, "centerPoint position: %f %f %f", euro_pallet_pos.x, euro_pallet_pos.y, euro_pallet_pos.z);

  // Move the end-effector above the ground
  const auto pick_up_pose = [&euro_pallet_pos]
  {
    geometry_msgs::msg::Pose msg;

    msg.orientation.x = 0.0;
    msg.orientation.y = std::sin(Constants::PI / 2);
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 2);

    msg.position.x = euro_pallet_pos.x;
    msg.position.y = euro_pallet_pos.y;
    msg.position.z = euro_pallet_pos.z + 0.06; //+ 0.06;
    return msg;
  }();
  MoveToPose(pick_up_pose, move_group_interface, moveit_visual_tools, logger);

  // Clear path constraints
  move_group_interface.clearPathConstraints();

  // Move to the home position
  MoveToHome(move_group_interface, moveit_visual_tools, logger);
}
