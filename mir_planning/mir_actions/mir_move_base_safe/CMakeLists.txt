cmake_minimum_required(VERSION 3.12)
project(mir_move_base_safe)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
  src/move_base_safe.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

add_executable(move_base_test
  src/move_base_safe.cpp
  src/move_base_safe_test.cpp
)

ament_target_dependencies(move_base_test
  ${dependencies}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS move_base_test ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY
 bt/
 DESTINATION share/${PROJECT_NAME}/bt
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})

ament_package()
