cmake_minimum_required(VERSION 3.5)
project(hric_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED) #for Time 或 Duration 这样的内置类型
find_package(nav_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(geographic_msgs)
find_package(action_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavToPose.action"
  "action/NavThroughPoses.action"
  "msg/LocStatus.msg"
  "msg/MarkerInfo.msg"
  "msg/MotionStatus.msg"
  "msg/FloatBaseRPYZ.msg"
  "msg/NavStatus.msg"
  "msg/TaskStatus.msg"
  "msg/DynamicObject.msg"
  "msg/DynamicObjectArray.msg"
  "msg/ObjectState.msg"
  "msg/ObjectSemantic.msg"
  "msg/ObjectShape.msg"
  "msg/TrackingStatus.msg"
  "msg/Object.msg"
  "msg/ObjectArray.msg"
  "msg/KeypointMatch.msg"
  "msg/KeypointMatches.msg"
  "msg/TrajectoryPoint.msg"
  "msg/PlannedTrajectory.msg"
  "srv/ListMarkers.srv"
  "srv/MarkAtPose.srv"
  "srv/MarkHere.srv"
  "srv/OperateMarker.srv"
  "srv/Relocate.srv"
  "srv/VisualRelocate.srv"
  "srv/SetGoal.srv"
  "srv/UpdateMarker.srv"
  "srv/SetMotionMode.srv"
  "srv/SetMotionNumber.srv"
  "srv/SetVisualLocMode.srv"
  "srv/StartCollectData.srv"
  "srv/StopCollectData.srv"
  "srv/StartCreateMap.srv"
  "srv/StartUploadData.srv"
  "srv/StopCreateMap.srv"
  "srv/StopUploadData.srv"
  "srv/StartNav.srv"
  "srv/StopNav.srv"
  "srv/StartMoveRotate.srv"
  "srv/StopMoveRotate.srv"
  "srv/LoadMap.srv"
  "srv/RequestResponse.srv"
  "srv/StartPreciseVelControl.srv"
  "srv/StartNormalVelControl.srv"
  "srv/StopVelControl.srv"
  "srv/ManageControllerStatus.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs vision_msgs nav_msgs 
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
