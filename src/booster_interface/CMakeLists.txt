cmake_minimum_required(VERSION 3.8)
project(booster_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AsrChunk.msg"
  "msg/BatteryState.msg"
  "msg/BoosterApiReqMsg.msg"
  "msg/BoosterApiRespMsg.msg"
  "msg/ButtonEventMsg.msg"
  "msg/MotorState.msg"
  "msg/MotorCmd.msg"
  "msg/LowCmd.msg"
  "msg/ImuState.msg"
  "msg/LowState.msg"
  "msg/RawBytesMsg.msg"
  "msg/Odometer.msg"
  "msg/FallDownState.msg"
  "msg/RemoteControllerState.msg"
  "msg/HandActionStatus.msg"
  "msg/HandCommand.msg"
  "msg/HandDdsMsg.msg"
  "msg/HandParam.msg"
  "msg/HandReplyData.msg"
  "msg/HandReplyParam.msg"
  "msg/HandTouchData.msg"
  "msg/HandTouchParam.msg"
  "msg/LightControlMsg.msg"
  "msg/LightPixel.msg"
  "msg/ProneBodyControlStatus.msg"
  "msg/RobocupBehaviorStatus.msg"
  "msg/RobotDdsBatteryStatus.msg"
  "msg/RobotDdsImuStatus.msg"
  "msg/RobotDdsJointStatus.msg"
  "msg/RobotProcessState.msg"
  "msg/RobotProcessStateMsg.msg"
  "msg/RobotStatesMsg.msg"
  "msg/RobotStatusDdsMsg.msg"
  "msg/RobotReplayTrajID.msg"
  "msg/TrainedTrajStatus.msg"
  "msg/Subtitle.msg"
  "srv/RpcService.srv"
  "srv/AgentService.srv"
)

install(DIRECTORY include/
        DESTINATION include/
)

macro(default_install_none)
endmacro()

macro(custom_install_env)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
endmacro()


ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
