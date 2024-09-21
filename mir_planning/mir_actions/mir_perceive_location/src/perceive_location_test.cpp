#include "mir_perceive_location/perceive_location.hpp"
#include "mir_common_actions/check_if_base_centered.hpp"
#include "mir_common_actions/move_arm_to_pose.hpp"
#include "mir_common_actions/grasp_action.hpp"
#include "mir_move_base_safe/move_base_safe.hpp"

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // get string from input arguments
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " <location>\n";
    return 1;
  }

  std::string location = argv[1];

  std::cout << "location: " << location << "\n";

  // create a node
  auto node = std::make_shared<rclcpp::Node>("PerceiveLocationAction");

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<PerceiveLocationAction>("PerceiveLocationAction",
                                                   node);

  factory.registerNodeType<CheckIfBaseCenteredAction>("CheckIfBaseCenteredAction",
                                                      node);

  factory.registerNodeType<MoveBaseSafeAction>("MirMoveBaseSafeAction", node);

  factory.registerNodeType<MoveArmToPoseAction>("MoveArmToPoseAction", node);

  factory.registerNodeType<GraspAction>("GraspAction", node);

  // get xml file
  std::string pkg_path =
    ament_index_cpp::get_package_share_directory("mir_perceive_location");

  std::string bt_path = pkg_path + "/bt/";

  // load all xml files in this directory
  for (const auto& entry : std::filesystem::directory_iterator(bt_path)) {
    if (entry.path().extension() == ".xml") {
      factory.registerBehaviorTreeFromFile(entry.path().string());
    }
  }

  BT::Tree tree = factory.createTree("MainTree");

  // tick the root of the tree
  // tree.tickWhileRunning();

  std::cout << "--- ticking\n";
  auto status = tree.tickOnce();
  std::cout << "--- status: " << toStr(status) << "\n\n";

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    // Sleep to avoid busy loops.
    // do NOT use other sleep functions!
    // Small sleep time is OK, here we use a large one only to
    // have less messages on the console.
    tree.sleep(std::chrono::milliseconds(1));

    // spin
    rclcpp::spin_some(node);

    std::cout << "--- ticking\n";
    status = tree.tickOnce();
    std::cout << "--- status: " << toStr(status) << "\n\n";
  }

  rclcpp::shutdown();

  return 0;
}