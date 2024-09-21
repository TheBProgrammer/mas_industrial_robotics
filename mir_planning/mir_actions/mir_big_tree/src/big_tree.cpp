#include "mir_big_tree/big_tree.hpp"

#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include <fstream>

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // create a node
  auto node = std::make_shared<rclcpp::Node>("BigTree");

  BT::BehaviorTreeFactory factory;

  // register actions
  factory.registerNodeType<PerceiveLocationAction>("PerceiveLocationAction", node);
  factory.registerNodeType<PickObjectAction>("PickObjectAction", node);

  std::string xml_models = BT::writeTreeNodesModelXML(factory);

  // get xml file
  std::string pkg_path =
    ament_index_cpp::get_package_share_directory("mir_big_tree");

  std::string bt_path = pkg_path + "/bt/";

  // load all xml files in this directory
  for (const auto& entry : std::filesystem::directory_iterator(bt_path)) {
    if (entry.path().extension() == ".xml") {
      factory.registerBehaviorTreeFromFile(entry.path().string());
    }
  }

  BT::Tree tree = factory.createTree("MainTree");

  std::cout << "----------- XML file  ----------\n"
            << BT::WriteTreeToXML(tree, false, false)
            << "--------------------------------\n";

  const unsigned port = 1667;
  BT::Groot2Publisher publisher(tree, port);

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