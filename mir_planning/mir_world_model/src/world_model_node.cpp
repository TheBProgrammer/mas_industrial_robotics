#include "mir_world_model/world_model_node.hpp"

WorldModelNode::WorldModelNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("world_model_node", options)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode created");

  // get node parameter overrides
  auto overrides = this->get_node_options().parameter_overrides();

  // declare overridable parameters
  for (auto& override : overrides) {
    declare_parameter(override.get_name(), override.get_parameter_value());
  }
}

WorldModelNode::~WorldModelNode()
{
  RCLCPP_INFO(get_logger(), "WorldModelNode destroyed");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode configured");

  // world model initialization
  world_model_ = std::make_shared<WorldModel>();

  // create wokstation objects from parameters
  auto workstation_names =
    this->list_parameters({ "workstations" }, 2).prefixes;
  for (auto& workstation_name : workstation_names) {
    std::string name =
      this->get_parameter(workstation_name + ".name").as_string();
    std::string type =
      this->get_parameter(workstation_name + ".type").as_string();
    double height =
      this->get_parameter(workstation_name + ".height").as_double();
    world_model_->addWorkstation(name, type, height);
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode activated");

  // create subscibers
  object_list_subscriber_ =
    this->create_subscription<mir_interfaces::msg::ObjectList>(
      "~/object_list",
      rclcpp::SensorDataQoS(),
      std::bind(
        &WorldModelNode::objectListCallback, this, std::placeholders::_1));

  // create action server
  object_selector_action_server_ =
    rclcpp_action::create_server<mir_interfaces::action::ObjectSelector>(
      shared_from_this(),
      "~/object_selector",
      std::bind(
        &WorldModelNode::objectSelectorHandleCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &WorldModelNode::objectSelectorCancelCallback, this,
        std::placeholders::_1),
      std::bind(
        &WorldModelNode::objectSelectorAcceptedCallback, this,
        std::placeholders::_1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode deactivated");

  LifecycleNode::on_deactivate(state);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_cleanup(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode cleaned up");

  LifecycleNode::on_cleanup(state);

  // destroy subscribers
  object_list_subscriber_.reset();
  
  // destroy action server
  object_selector_action_server_.reset();

  // destroy world model
  world_model_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_shutdown(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode shut down");

  LifecycleNode::on_shutdown(state);

  // delete subscribers
  delete object_list_subscriber_.get();

  // delete action server
  delete object_selector_action_server_.get();

  // destroy world model
  delete world_model_.get();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse
WorldModelNode::objectSelectorHandleCallback(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const mir_interfaces::action::ObjectSelector::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode received object selector goal");

  // check if goal is valid
  if (goal->obj_name.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
WorldModelNode::objectSelectorCancelCallback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<
    mir_interfaces::action::ObjectSelector>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
WorldModelNode::objectSelectorAcceptedCallback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<
    mir_interfaces::action::ObjectSelector>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode accepted object selector goal");

  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&WorldModelNode::executeObjectSelector, this, std::placeholders::_1), goal_handle}.detach();
}

void
WorldModelNode::executeObjectSelector(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<
    mir_interfaces::action::ObjectSelector>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode executing object selector goal");

  // helper variables
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<mir_interfaces::action::ObjectSelector::Result>();

  // get object from world model
  mir_interfaces::msg::Object object;
  world_model_->getWorkstationObject(goal->workstation_name, goal->obj_name, object);

  if (object.name.empty()) {
    RCLCPP_ERROR(get_logger(), "Object not found");
    result->success = false;
    goal_handle->succeed(result);
    return;
  }

  // check if goal is done
  if (rclcpp::ok()) {
    result->success = true;
    result->obj = object;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Object selector goal succeeded");
  }
}

void
WorldModelNode::objectListCallback(
  const mir_interfaces::msg::ObjectList::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode received object list");

  // update world model
  world_model_->addObjectToWorkstation(msg);
}

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(WorldModelNode)