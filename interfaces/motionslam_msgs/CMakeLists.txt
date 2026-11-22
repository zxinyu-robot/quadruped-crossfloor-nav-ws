cmake_minimum_required(VERSION 3.8)
project(motionslam_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RuntimeEnvelope.msg"
  "msg/PosePoint.msg"
  "msg/TrajectoryProbe.msg"
  "msg/MotionCommand.msg"
  "msg/TerrainObservation.msg"
  "msg/TerrainDecision.msg"
  "msg/WorldRef.msg"
  "msg/TaskBinding.msg"
  "msg/RebindRequest.msg"
  "msg/RebindRecord.msg"
  "msg/RebindState.msg"
  "msg/ArtifactEpoch.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
