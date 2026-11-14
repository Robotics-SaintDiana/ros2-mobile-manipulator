cmake_minimum_required(VERSION 3.8)
project(robot_bt_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_manipulation_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(behaviortree_cpp_v3 REQUIRED)
if(TARGET behaviortree_cpp_v3::behaviortree_cpp_v3)
  set(BT_TARGET behaviortree_cpp_v3::behaviortree_cpp_v3)
elseif(TARGET BT::behaviortree_cpp_v3)
  find_package(Boost REQUIRED COMPONENTS coroutine)
  find_package(ament_index_cpp REQUIRED)
  set(BT_TARGET BT::behaviortree_cpp_v3)
else()
  message(FATAL_ERROR "behaviortree_cpp_v3 found, but no usable imported target")
endif()

include_directories(include)

add_library(bt_nodes SHARED
  src/nodes/move_home.cpp
  src/nodes/get_object_center.cpp
)

target_compile_features(bt_nodes PUBLIC cxx_std_17)

target_link_libraries(bt_nodes
  ${BT_TARGET}
)

ament_target_dependencies(bt_nodes
  rclcpp
  rclcpp_action
  robot_manipulation_msgs
  geometry_msgs
  sensor_msgs
)

add_executable(bt_manager_node src/bt_manager_node.cpp)

target_compile_features(bt_manager_node PUBLIC cxx_std_17)

target_link_libraries(bt_manager_node bt_nodes)

ament_target_dependencies(bt_manager_node
  rclcpp
  rclcpp_action
  ament_index_cpp
)

install(TARGETS bt_manager_node bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()