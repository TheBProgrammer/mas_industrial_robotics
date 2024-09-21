#include "mir_move_base_safe/move_base_safe.hpp"

MoveBaseSafeAction::MoveBaseSafeAction(const std::string& name,
                                       const BT::NodeConfiguration& config,
                                       const std::shared_ptr<rclcpp::Node>& node)
  : BT::StatefulActionNode(name, config), _node(node)
{
  // _action_client = rclcpp_action::create_client<mir_interfaces::action::ObjectDetection>(
  //   _node, "object_detection");
}

BT::PortsList
MoveBaseSafeAction::providedPorts()
{
  return { BT::InputPort<std::string>("location") };
}

// invoked once at the beginning
BT::NodeStatus
MoveBaseSafeAction::onStart()
{
  // read input port
  BT::Expected<std::string> location = getInput<std::string>("location");

  if (!location) {
    throw BT::RuntimeError("missing required input [location]: ", location.error());
  }

  RCLCPP_INFO(_node->get_logger(), "MoveBaseSafeAction: location: %s", location.value().c_str());

  // wait for action server
  // if (!_action_client->wait_for_action_server()) {
  //   RCLCPP_ERROR(
  //     _node->get_logger(),
  //     "MoveBaseSafeAction: action server not available after waiting");
  //   return BT::NodeStatus::FAILURE;
  // }

  // send goal
  // auto goal_msg = mir_interfaces::action::ObjectDetection::Goal();
  // goal_msg.obj_category = "atwork";
  // goal_msg.obj_name = "";
  // goal_msg.workstation = workstation_name.value();

  // RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: sending goal");

  // goal_handle_future = _action_client->async_send_goal(goal_msg);

  return BT::NodeStatus::RUNNING;
}

// If onStart() returned RUNNING, we will keep calling
// this method until it return something different from RUNNING
BT::NodeStatus
MoveBaseSafeAction::onRunning()
{
  // RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: onRunning");

  // if (!goal_handle_future.valid()) {
  //   RCLCPP_ERROR(_node->get_logger(), "PerceiveLocationAction: invalid future");
  //   return BT::NodeStatus::FAILURE;
  // }

  // auto status = goal_handle_future.wait_for(std::chrono::seconds(0));

  // if (status == std::future_status::ready) {
  //   auto goal_handle = goal_handle_future.get();

  //   if (!goal_handle) {
  //     RCLCPP_ERROR(_node->get_logger(), "PerceiveLocationAction: goal was rejected by server");
  //     return BT::NodeStatus::FAILURE;
  //   }

  //   if (goal_handle->get_status() == action_msgs::msg::GoalStatus::STATUS_ACCEPTED) {
  //     RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: goal accepted");
  //     isGoalAccepted = true;
  //   } else {
  //     RCLCPP_ERROR(_node->get_logger(), "PerceiveLocationAction: goal rejected");
  //     return BT::NodeStatus::FAILURE;
  //   }
  // }

  // if (isGoalAccepted) {
  //   auto result_future = _action_client->async_get_result(goal_handle_future.get());

  //   if (result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
  //     auto result = result_future.get();

  //     if (!result) {
  //       RCLCPP_ERROR(_node->get_logger(), "PerceiveLocationAction: failed to get result");
  //       return BT::NodeStatus::FAILURE;
  //     }

  //     if (result->result->obj_found) {
  //       RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: object found");
  //       return BT::NodeStatus::SUCCESS;
  //     } else {
  //       RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: object not found");
  //       return BT::NodeStatus::FAILURE;
  //     }
  //   }
  // }

  return BT::NodeStatus::RUNNING;
}

// callback to execute if the action was aborted by another node
void
MoveBaseSafeAction::onHalted()
{
  RCLCPP_INFO(_node->get_logger(), "MoveBaseSafeAction: halted");
}