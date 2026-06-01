cmake_minimum_required(VERSION 3.8)
project(megajaw_hardware_direct)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Create library: driver c files -> .so
add_library(${PROJECT_NAME} SHARED
  src/MotorDriver.cpp
  src/megajaw_direct_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

# Link dependancies to pigpio and pthread
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(${PROJECT_NAME} pigpiod_if2 pthread)

# Plugin export
pluginlib_export_plugin_description_file(hardware_interface megajaw_hardware_direct.xml)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES megajaw_hardware_direct.xml
  DESTINATION share/${PROJECT_NAME}
)



ament_package()