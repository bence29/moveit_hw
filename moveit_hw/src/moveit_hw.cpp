#include <cmath>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_hw/utils.hpp"
/* #include "moveit_hw/first_scenario.hpp"
#include "moveit_hw/second_scenario.hpp" */
#include "moveit_hw/palettazas.hpp"

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>(
      "moveit_hw",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)); //Magától deklarálja a paramétereket

  // Create a ROS logger
  const auto logger = rclcpp::get_logger("moveit_hw");

  // Create an executor so that the visual tools have access to the event system
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]()
                             { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "manipulator");
  move_group_interface.setPlanningPipelineId("chomp"); //planner fajtája ompl
  move_group_interface.setPlannerId("CHOMP");  //RRTConnectkConfigDefault

  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools(
      node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel());
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.trigger();

  RCLCPP_INFO(logger, "Planning frame: %s\tEnd-effector link: %s",
    move_group_interface.getPlanningFrame().c_str(), move_group_interface.getEndEffectorLink().c_str());

  //Delete previous scenes
  move_group_interface.detachObject();
  ClearScene(moveit_visual_tools);
  // Setup the scene
  Setup();
  MoveToHome(move_group_interface, logger);

  // Palettazes ompl-el
  palettazas(move_group_interface, moveit_visual_tools, logger, node);
  ClearScene(moveit_visual_tools);
  MoveToHome(move_group_interface, logger);

  // Edit the MoveIt MoveGroup Interface
/*   move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "manipulator");
  move_group_interface.setPlanningPipelineId("chomp"); //planner fajtája
  move_group_interface.setPlannerId("CHOMP");

  moveit_visual_tools = moveit_visual_tools::MoveItVisualTools(
      node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel());
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.trigger();

  RCLCPP_INFO(logger, "Planning frame: %s\tEnd-effector link: %s",
    move_group_interface.getPlanningFrame().c_str(), move_group_interface.getEndEffectorLink().c_str());

  // Palettazes chomp-al
  palettazas(move_group_interface, moveit_visual_tools, logger, node);
  ClearScene(moveit_visual_tools);
  MoveToHome(move_group_interface, logger); */

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
