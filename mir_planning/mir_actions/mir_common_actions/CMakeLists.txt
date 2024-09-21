cmake_minimum_required(VERSION 3.8)
project(mir_common_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(mir_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp
  std_msgs
  mir_interfaces
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/align_to_pose.cpp
  src/check_if_base_centered.cpp
  src/grasp_action.cpp
  src/move_arm.cpp
  src/move_arm_to_pose.cpp
  src/verify_grasp_action.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
