#include "mir_pick_object/select_object.hpp"

SelectObjectAction::SelectObjectAction(const std::string& name,
                                         const BT::NodeConfiguration& config,
                                         const std::shared_ptr<rclcpp::Node>& node)
: BT::StatefulActionNode(name, config), _node(node)
{}

BT::PortsList SelectObjectAction::providedPorts()
{
  return{ BT::InputPort<std::string>("obj_name"),
          BT::InputPort<std::string>("workstation_name")};
}

BT::NodeStatus SelectObjectAction::onStart()
{
  // read ports
  std::string obj_name;
  std::string workstation_name;

  if (!getInput<std::string>("obj_name", obj_name))
  {
    throw BT::RuntimeError("missing required input [obj_name]: ", obj_name);
  }

  if (!getInput<std::string>("workstation_name", workstation_name))
  {
    throw BT::RuntimeError("missing required input [workstation_name]: ", workstation_name);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SelectObjectAction::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void SelectObjectAction::onHalted()
{
  // cleanup any state change
}