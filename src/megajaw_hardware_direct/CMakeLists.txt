cmake_minimum_required(VERSION 3.8)
project(megajaw_hardware_direct)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/MotorDriver.cpp
  src/megajaw_direct_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  pigpio
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface megajaw_hardware_direct.xml)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES megajaw_hardware_direct.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()