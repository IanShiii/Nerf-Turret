cmake_minimum_required(VERSION 3.8)
project(turret_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(turret_controller SHARED src/turret_controller.cpp)
target_include_directories(turret_controller PUBLIC include)
ament_target_dependencies(turret_controller rclcpp controller_interface hardware_interface pluginlib rclcpp_lifecycle std_msgs std_srvs)
pluginlib_export_plugin_description_file(controller_interface turret_controller.xml)

install(TARGETS turret_controller
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
