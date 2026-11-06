cmake_minimum_required(VERSION 3.8)
project(rcm_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoState.msg" "msg/ArmState.msg" "msg/TendonGoal.msg"
  "msg/BendCommand.msg" "msg/BendState.msg"
  "srv/SetTorque.srv" "srv/SetZero.srv" "srv/StartTest.srv" "srv/Respond.srv"
  DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
