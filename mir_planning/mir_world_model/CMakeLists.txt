cmake_minimum_required(VERSION 3.12)
project(mir_world_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(mir_interfaces REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${mir_interfaces_INCLUDE_DIRS}
)

add_library(world_model_node SHARED
  src/world_model_node.cpp
  src/world_model.cpp
)

ament_target_dependencies(world_model_node
  rclcpp
  rclcpp_lifecycle 
  rclcpp_components
  rclcpp_action
  std_msgs 
  geometry_msgs 
  lifecycle_msgs 
  mir_interfaces
)

target_link_libraries(world_model_node
  ${rclcpp_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${lifecycle_msgs_LIBRARIES}
  ${mir_interfaces_LIBRARIES}
)

target_compile_definitions(world_model_node
            PRIVATE "COMPOSITION_BUILDING_DLL")

rclcpp_components_register_nodes(world_model_node "WorldModelNode")
set(node_plugins "${node_plugins}WorldModelNode;$<TARGET_FILE:world_model_node>\n")

install(TARGETS
  world_model_node

  EXPORT export_world_model_node

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
 config/
 DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
