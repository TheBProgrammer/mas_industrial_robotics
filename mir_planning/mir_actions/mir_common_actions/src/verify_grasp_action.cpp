#include "mir_common_actions/verify_grasp_action.hpp"

VerifyGraspAction::VerifyGraspAction(const std::string& name,
                         const BT::NodeConfiguration& config,
                         const std::shared_ptr<rclcpp::Node>& node)
: BT::StatefulActionNode(name, config), _node(node)
{}

BT::PortsList VerifyGraspAction::providedPorts()
{
  return{ BT::InputPort<std::string>("command") };
}

BT::NodeStatus VerifyGraspAction::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VerifyGraspAction::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void VerifyGraspAction::onHalted()
{
  // cleanup any state change
}