cmake_minimum_required(VERSION 3.8)
project(example_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(booster_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(EXECUTABLES
  client
  arm_sample
  double_arm
  keyboard
)

foreach(target ${EXECUTABLES})
  add_executable(${target}  src/${target}.cpp)
  ament_target_dependencies(${target} 
    rclcpp 
    std_msgs
    booster_interface
    rosidl_default_runtime)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(PROGRAMS
  example_package/client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME client_py
)

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
