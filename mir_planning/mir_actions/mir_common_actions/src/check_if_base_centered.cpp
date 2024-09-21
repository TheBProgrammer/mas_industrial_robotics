#include "mir_common_actions/check_if_base_centered.hpp"

CheckIfBaseCenteredAction::CheckIfBaseCenteredAction(const std::string& name,
                                                   const BT::NodeConfiguration& config,
                                                   const std::shared_ptr<rclcpp::Node>& node)
  : BT::StatefulActionNode(name, config), _node(node)
{
}

BT::PortsList
CheckIfBaseCenteredAction::providedPorts()
{
  return { BT::InputPort<std::string>("location") };
}

// invoked once at the beginning
BT::NodeStatus
CheckIfBaseCenteredAction::onStart()
{
  // read input port
  BT::Expected<std::string> location = getInput<std::string>("location");

  if (!location) {
    throw BT::RuntimeError("missing required input [location]: ", location.error());
  }

  RCLCPP_INFO(_node->get_logger(), "CheckIfBaseCenteredAction: location: %s", location.value().c_str());

  return BT::NodeStatus::RUNNING;
}

// If onStart() returned RUNNING, we will keep calling
// this method until it return something different from RUNNING
BT::NodeStatus
CheckIfBaseCenteredAction::onRunning()
{
  RCLCPP_INFO(_node->get_logger(), "CheckIfBaseCenteredAction: onRunning");

  return BT::NodeStatus::SUCCESS;
}

// callback to execute if the action was aborted by another node
void
CheckIfBaseCenteredAction::onHalted()
{
  RCLCPP_INFO(_node->get_logger(), "CheckIfBaseCenteredAction: halted");
}