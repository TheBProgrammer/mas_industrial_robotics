#include "mir_common_actions/grasp_action.hpp"

GraspAction::GraspAction(const std::string& name,
                         const BT::NodeConfiguration& config,
                         const std::shared_ptr<rclcpp::Node>& node)
: BT::StatefulActionNode(name, config), _node(node)
{}

BT::PortsList GraspAction::providedPorts()
{
  return{ BT::InputPort<std::string>("command") };
}

BT::NodeStatus GraspAction::onStart()
{
  // get command from blackboard
  std::string command;
  if (!getInput<std::string>("command", command))
  {
    throw BT::RuntimeError("missing required input [command]: ", command);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GraspAction::onRunning()
{
  // get command from blackboard
  std::string command;
  if (!getInput<std::string>("command", command))
  {
    throw BT::RuntimeError("missing required input [command]: ", command);
  }
  return BT::NodeStatus::SUCCESS;
}

void GraspAction::onHalted()
{
  // cleanup any state change
}