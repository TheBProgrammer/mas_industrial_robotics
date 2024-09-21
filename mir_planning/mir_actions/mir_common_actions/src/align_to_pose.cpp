#include "mir_common_actions/align_to_pose.hpp"

AlignToPoseAction::AlignToPoseAction(const std::string& name,
                                         const BT::NodeConfiguration& config,
                                         const std::shared_ptr<rclcpp::Node>& node)
: BT::StatefulActionNode(name, config), _node(node)
{}

BT::PortsList AlignToPoseAction::providedPorts()
{
  return{ BT::InputPort<std::string>("pose") };
}

BT::NodeStatus AlignToPoseAction::onStart()
{
  // get pose from blackboard
  std::string pose;
  if (!getInput<std::string>("pose", pose))
  {
    throw BT::RuntimeError("missing required input [pose]: ", pose);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignToPoseAction::onRunning()
{
  // get pose from blackboard
  std::string pose;
  if (!getInput<std::string>("pose", pose))
  {
    throw BT::RuntimeError("missing required input [pose]: ", pose);
  }
  return BT::NodeStatus::SUCCESS;
}

void AlignToPoseAction::onHalted()
{
  // cleanup any state change
}