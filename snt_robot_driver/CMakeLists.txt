cmake_minimum_required(VERSION 3.8)
project(snt_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)

# src/snt_robot_driver.cpp 코드 라이브러리를 snt_hw 이름으로 생성
# snt_robot_driver.xml을 package root에  만들때 경로를 snt_hw로 입력해주어야  함! 

add_library(snt_hw SHARED src/snt_robot_driver.cpp)

ament_target_dependencies(snt_hw
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_manager
  pluginlib
)

# export library for pluginlib

install(TARGETS snt_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# CMakeLists.txt에서 Export 하는 과정 누락

pluginlib_export_plugin_description_file(hardware_interface snt_robot_driver.xml)
install(FILES snt_robot_driver.xml DESTINATION share/${PROJECT_NAME})

ament_package()