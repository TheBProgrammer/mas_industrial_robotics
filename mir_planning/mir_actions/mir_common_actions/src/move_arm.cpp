#include "mir_common_actions/move_arm.hpp"

MoveArmAction::MoveArmAction(const std::string& name,
                                         const BT::NodeConfiguration& config,
                                         const std::shared_ptr<rclcpp::Node>& node)
: BT::StatefulActionNode(name, config), _node(node)
{}

BT::PortsList MoveArmAction::providedPorts()
{
  return{ BT::InputPort<std::string>("direction") };
}

BT::NodeStatus MoveArmAction::onStart()
{
  // get pose from blackboard
  std::string direction;
  if (!getInput<std::string>("direction", direction))
  {
    throw BT::RuntimeError("missing required input [direction]: ", direction);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveArmAction::onRunning()
{
  // get pose from blackboard
  std::string direction;
  if (!getInput<std::string>("direction", direction))
  {
    throw BT::RuntimeError("missing required input [direction]: ", direction);
  }
  return BT::NodeStatus::SUCCESS;
}

void MoveArmAction::onHalted()
{
  // cleanup any state change
}