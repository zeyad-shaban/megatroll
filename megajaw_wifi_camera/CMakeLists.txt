cmake_minimum_required(VERSION 3.8)
project(megajaw_wifi_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_driver_node
  src/camera_driver_node.cpp
)

target_include_directories(camera_driver_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(camera_driver_node
  rclcpp
  sensor_msgs
)

target_link_libraries(camera_driver_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS camera_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
