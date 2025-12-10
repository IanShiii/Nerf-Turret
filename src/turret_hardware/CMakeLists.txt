cmake_minimum_required(VERSION 3.8)
project(turret_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PiPCA9685 REQUIRED)

add_library(turret_hardware SHARED src/turret_real_hardware.cpp src/turret_sim_hardware.cpp)
target_include_directories(turret_hardware PUBLIC include)
ament_target_dependencies(turret_hardware hardware_interface pluginlib rclcpp rclcpp_lifecycle PiPCA9685)
target_link_libraries(turret_hardware
  wiringPi
  PiPCA9685::PiPCA9685
)

pluginlib_export_plugin_description_file(hardware_interface turret_hardware.xml)

install(TARGETS turret_hardware
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
