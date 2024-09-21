cmake_minimum_required(VERSION 3.8)
project(mir_perceive_location)

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
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mir_interfaces REQUIRED)
find_package(mir_common_actions REQUIRED)
find_package(mir_move_base_safe REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp
  std_msgs
  mir_interfaces
  mir_common_actions
  mir_move_base_safe
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/perceive_location.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

add_executable(perceive_location
  src/perceive_location.cpp
  src/perceive_location_test.cpp
)

ament_target_dependencies(perceive_location
  ${dependencies}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS perceive_location ${PROJECT_NAME}
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
