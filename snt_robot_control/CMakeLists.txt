cmake_minimum_required(VERSION 3.8)
project(snt_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(snt_fjt_server src/snt_fjt_server.cpp)
add_executable(snt_traj_check src/snt_traj_check.cpp)

# snt_fjt_server가 의존하는 항목 

ament_target_dependencies(snt_fjt_server rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs)

install(TARGETS snt_fjt_server
  DESTINATION lib/${PROJECT_NAME}
)

# snt_traj_check가 의존하는 항목 

ament_target_dependencies(snt_traj_check rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs)

install(TARGETS snt_traj_check
  DESTINATION lib/${PROJECT_NAME}
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
