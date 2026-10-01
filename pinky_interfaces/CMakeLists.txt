cmake_minimum_required(VERSION 3.8)
project(pinky_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/FleetPermit.msg"
  "msg/RobotHeartbeat.msg"
  "srv/Emotion.srv"
  "srv/SetBrightness.srv"
  "srv/SetLamp.srv"
  "srv/SetLed.srv"
  "action/FollowLane.action"
  DEPENDENCIES action_msgs builtin_interfaces std_msgs
)

ament_package()
