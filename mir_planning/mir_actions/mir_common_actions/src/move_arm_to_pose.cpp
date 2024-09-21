#include "mir_common_actions/move_arm_to_pose.hpp"

MoveArmToPoseAction::MoveArmToPoseAction(const std::string& name,
                                         const BT::NodeConfiguration& config,
                                         const std::shared_ptr<rclcpp::Node>& node)
: BT::StatefulActionNode(name, config), _node(node)
{}

BT::PortsList MoveArmToPoseAction::providedPorts()
{
  return{ BT::InputPort<std::string>("pose") };
}

BT::NodeStatus MoveArmToPoseAction::onStart()
{
  // get pose from blackboard
  std::string pose;
  if (!getInput<std::string>("pose", pose))
  {
    throw BT::RuntimeError("missing required input [pose]: ", pose);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveArmToPoseAction::onRunning()
{
  // get pose from blackboard
  std::string pose;
  if (!getInput<std::string>("pose", pose))
  {
    throw BT::RuntimeError("missing required input [pose]: ", pose);
  }
  return BT::NodeStatus::SUCCESS;
}

void MoveArmToPoseAction::onHalted()
{
  // cleanup any state change
}