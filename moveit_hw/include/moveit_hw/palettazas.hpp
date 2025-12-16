#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_hw/utils.hpp"

#include <string>

void SetupScene(const rclcpp::Node::SharedPtr &node) //A node-ot végig át kellett adni a függvényeken, hogy kiolvasható legyen a yaml-ből a paraméter
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
    const double L = 1.2;   // hossz
    const double W = 0.8;   // szélesség
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
    const double placementWidth = node->get_parameter("placementWidth_p").as_double();
    const double placementLength = node->get_parameter("placementLength_p").as_double();
    const double placementHeight = node->get_parameter("placementHeight_p").as_double();
    for(int j = 1; j < 17; j++) {
      double x_pl_offs = - L/2 + placementLength/2 + ((j-1)%4) * placementLength;
      //y offszetek állítása:
      double y_pl_offs = 0;
      if(j < 5)
        y_pl_offs = - W/2 + placementWidth/2;
      else if(j < 9)
        y_pl_offs = - W/2 + placementWidth/2 + placementWidth;
      else if(j < 13)
        y_pl_offs = - W/2 + placementWidth/2 + 2*+ placementWidth;
      else
        y_pl_offs = - W/2 + placementWidth/2 + 3*+ placementWidth;
      
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

void newBox(const rclcpp::Node::SharedPtr &node, int iterator) {
  const auto [object, color] = [node, iterator]() 
  {  
    moveit_msgs::msg::CollisionObject object;
    //paraméterek
    const double Box_xcoord = node->get_parameter("Box_xcoord_p").as_double();
    const double Box_ycoord = node->get_parameter("Box_ycoord_p").as_double();
    const double Box_zcoord = node->get_parameter("Box_zcoord_p").as_double();
    const double Box_width = node->get_parameter("Box_width_p").as_double();
    const double Box_length = node->get_parameter("Box_length_p").as_double();
    const double Box_height = node->get_parameter("Box_height_p").as_double();

    object.id = ("box_" + std::to_string(iterator));
    object.header.frame_id = "world";

    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = { Box_length, Box_width, Box_height };

    object.primitive_poses.resize(1);
    object.primitive_poses[0].position.x = Box_xcoord;
    object.primitive_poses[0].position.y = Box_ycoord;
    object.primitive_poses[0].position.z = Box_zcoord;
    object.primitive_poses[0].orientation.w = 1.0;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    // Próba box színe
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 1.0;

    return std::make_pair(object, color);
  }();
  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(object, color);
}

void pickAndPlace(moveit::planning_interface::MoveGroupInterface &move_group_interface,
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
  move_group_interface.allowReplanning(true);
  // move_group_interface.setPlanningTime(20.0);
  move_group_interface.setReplanAttempts(30);
  move_group_interface.setLookAroundAttempts(50);
  move_group_interface.setNumPlanningAttempts(100);
  MoveToPose(box_pick_up_pose, move_group_interface, moveit_visual_tools, logger);

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
  move_group_interface.allowReplanning(true);
  // move_group_interface.setPlanningTime(20.0);
  move_group_interface.setReplanAttempts(30);
  move_group_interface.setLookAroundAttempts(50);
  move_group_interface.setNumPlanningAttempts(100);
  MoveToPose(put_down_pose, move_group_interface, moveit_visual_tools, logger);

  // Detach the box from the end-effector
  move_group_interface.detachObject("box_" + std::to_string(iterator));
  moveit_visual_tools.prompt("box_ " + std::to_string(iterator) + " detached from the end-effector. Press 'Next' to continue");
}

void palettazas(  //ez a "main"
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
  const rclcpp::Logger &logger, const rclcpp::Node::SharedPtr &node)
{
  moveit_visual_tools.prompt("Press 'Next' to start palettazas");

  SetupScene(node);

  for(int i = 1; i < 17; i++) { //Pick and place és box teremtő loop
  newBox(node, i);
  pickAndPlace(move_group_interface, moveit_visual_tools, logger, node, i);
  }

  // Clear path constraints
  move_group_interface.clearPathConstraints();

  // Move to the home position
  moveit_visual_tools.prompt("Press 'Next' to move to the home position");
  MoveToHome(move_group_interface, moveit_visual_tools, logger);
}
