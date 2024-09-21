#ifndef MIR_PERCEIVE_LOCATION__PERCEIVE_LOCATION_HPP_
#define MIR_PERCEIVE_LOCATION__PERCEIVE_LOCATION_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

#include <chrono>
#include <filesystem>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include "mir_interfaces/action/object_detection.hpp"

class PerceiveLocationAction : public BT::StatefulActionNode
{
public:
  PerceiveLocationAction(const std::string& name,
                         const BT::NodeConfiguration& config,
                         const std::shared_ptr<rclcpp::Node>& node);

  static BT::PortsList providedPorts();

  // invoked once at the beginning
  BT::NodeStatus onStart() override;

  // If onStart() returned RUNNING, we will keep calling
  // this method until it return something different from RUNNING
  BT::NodeStatus onRunning() override;

  // callback to execute if the action was aborted by another node
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Node> _node;

  bool isGoalAccepted = false;

  // action client
  rclcpp_action::Client<mir_interfaces::action::ObjectDetection>::SharedPtr
    _action_client;

  // future to store the goal handle
  std::shared_future<rclcpp_action::ClientGoalHandle<
    mir_interfaces::action::ObjectDetection>::SharedPtr>
    goal_handle_future;
};

#endif  // MIR_PERCEIVE_LOCATION__PERCEIVE_LOCATION_HPP_