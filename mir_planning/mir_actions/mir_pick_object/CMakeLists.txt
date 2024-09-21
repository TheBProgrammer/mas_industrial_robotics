cmake_minimum_required(VERSION 3.8)
project(mir_pick_object)

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
find_package(mir_perceive_location REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp
  std_msgs
  mir_interfaces
  mir_common_actions
  mir_perceive_location
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/pick_object.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

add_executable(pick_object
  src/pick_object.cpp
  src/select_object.cpp
  src/pick_object_test.cpp
)

ament_target_dependencies(pick_object
  ${dependencies}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS pick_object ${PROJECT_NAME}
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
