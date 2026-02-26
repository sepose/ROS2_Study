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

install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()