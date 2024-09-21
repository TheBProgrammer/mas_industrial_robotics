/**
 * @brief world_model ros node
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "mir_interfaces/action/object_selector.hpp"

#include "mir_interfaces/msg/object_list.hpp"

#include "mir_world_model/world_model.hpp"

class WorldModelNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit WorldModelNode(const rclcpp::NodeOptions& options);

  virtual ~WorldModelNode();

  // Transition callback for state configuring
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&);

  // Transition callback for state activating
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state);

  // Transition callback for state deactivating
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state);

  // Transition callback for state cleaningup
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&);

  // Transition callback for state shutting down
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state);

private:
  // ============================ Members ============================
  std::shared_ptr<WorldModel> world_model_;

  // subscribers
  rclcpp::Subscription<mir_interfaces::msg::ObjectList>::SharedPtr
    object_list_subscriber_;

  // object selector action server
  rclcpp_action::Server<mir_interfaces::action::ObjectSelector>::SharedPtr
    object_selector_action_server_;

  // subscribers callbacks
  void objectListCallback(const mir_interfaces::msg::ObjectList::SharedPtr msg);

  // action server callbacks
  rclcpp_action::GoalResponse objectSelectorHandleCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mir_interfaces::action::ObjectSelector::Goal> goal);

  rclcpp_action::CancelResponse objectSelectorCancelCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      mir_interfaces::action::ObjectSelector>> goal_handle);

  void objectSelectorAcceptedCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      mir_interfaces::action::ObjectSelector>> goal_handle);


  // ============================ Methods ============================
  void executeObjectSelector(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      mir_interfaces::action::ObjectSelector>> goal_handle);
};