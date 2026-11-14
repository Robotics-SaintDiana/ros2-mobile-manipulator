cmake_minimum_required(VERSION 3.8)
project(panda_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_manipulation_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(include)

add_executable(
  manipulator_task_server
  src/manipulator_task_server.cpp
  src/manipulator_executor.cpp
  src/perception_executor.cpp
  src/grasp_feasibility_filter.cpp
)

target_compile_features(manipulator_task_server PUBLIC cxx_std_17)

ament_target_dependencies(manipulator_task_server
  rclcpp
  rclcpp_action
  robot_manipulation_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_geometry_msgs
  moveit_ros_planning_interface
)

add_executable(
  robot_skills_server
  src/skill_server.cpp
  src/manipulator_executor.cpp
  src/perception_executor.cpp
  src/skills/move_home_server.cpp
  src/skills/get_object_center.cpp
)

target_compile_features(robot_skills_server PUBLIC cxx_std_17)

ament_target_dependencies(
  robot_skills_server
  rclcpp
  rclcpp_action
  robot_manipulation_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_geometry_msgs
  tf2_eigen
  moveit_ros_planning_interface
)

install(
  TARGETS manipulator_task_server robot_skills_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
