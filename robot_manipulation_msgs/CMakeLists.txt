cmake_minimum_required(VERSION 3.8)
project(robot_manipulation_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteBTTask.action"  
  "msg/ErrorCode.msg"
  "action/InferGrasps.action"

  "action/GetObjectCenter.action"
  "action/GetFusionCloud.action"
  "action/GetJointValues.action"
  "action/FilterGrasps.action"
  "action/ExecuteGrasp.action"

  "action/GetPlatePosition.action"
  "action/ExecutePlace.action"

  "action/PreparePickScene.action"
  "action/CleanupPickScene.action"
  "action/PreparePlaceScene.action"
  "action/CleanupPlaceScene.action"

  DEPENDENCIES action_msgs geometry_msgs std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
