cmake_minimum_required(VERSION 3.10)
project(aero_controller_msgs)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ScriptReqJNameInterf.msg"
    "msg/ScriptReqJNoInterf.msg"
    "msg/ScriptResJoint.msg"
    "msg/Status.msg"
    "msg/VersionMs.msg"
    "msg/StatusParsed.msg"
    "msg/StatusParsedRobotStatus.msg"
    "msg/StatusParsedMsMs.msg"
    "msg/StatusParsedPdbSwitch.msg"
    "msg/StatusParsedMsPc.msg"
    "msg/StatusParsedElectricalStatus.msg"
    "msg/StatusParsedPdbIo.msg"
    "msg/RobotStatus.msg"
    "msg/RobotStatusParsed.msg"
    "msg/RobotStatusParsedList.msg"
    "srv/GetAd.srv"
    "srv/GetVersion.srv"
    "srv/RunScript.srv"
    "srv/Servo.srv"
    "srv/SetCurrent.srv"
    "srv/CanThrough.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
