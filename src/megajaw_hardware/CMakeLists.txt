cmake_minimum_required(VERSION 3.8)  
project(megajaw_hardware)  
  
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")  
  add_compile_options(-Wall -Wextra -Wpedantic)  
endif()  
  
# find dependencies  
find_package(ament_cmake REQUIRED)  
find_package(hardware_interface REQUIRED)  
find_package(pluginlib REQUIRED)  
find_package(rclcpp REQUIRED)  
find_package(rclcpp_lifecycle REQUIRED)  
find_package(std_msgs REQUIRED)  
  
# Create library: driver c files -> .so  
add_library(megajaw_hardware SHARED  
  src/MotorDriver.cpp  
  src/megajaw_interface.cpp  
)  
  
target_include_directories(megajaw_hardware PRIVATE  
  include  
)  
  
# Link dependencies  
ament_target_dependencies(megajaw_hardware  
  hardware_interface  
  pluginlib  
  rclcpp  
  rclcpp_lifecycle  
)  
  
# Link pthread for serial communication (standard Linux termios API)  
target_link_libraries(megajaw_hardware pthread)  
  
# Plugin export  
pluginlib_export_plugin_description_file(hardware_interface megajaw_hardware.xml)  
  
# ─── Gripper control node ───────────────────────────────────  
add_executable(gripper_control_node  
  src/GripperDriver.cpp  
  src/gripper_control_node.cpp  
)  
  
target_include_directories(gripper_control_node PRIVATE  
  include  
)  
  
ament_target_dependencies(gripper_control_node  
  rclcpp  
  std_msgs  
)  
  
target_link_libraries(gripper_control_node pthread)  
  
# ─── Install ────────────────────────────────────────────────  
install(TARGETS megajaw_hardware  
  DESTINATION lib/${PROJECT_NAME}  
)  
  
install(TARGETS gripper_control_node  
  DESTINATION lib/${PROJECT_NAME}  
)  
  
install(DIRECTORY include/  
  DESTINATION include  
)  
  
install(FILES megajaw_hardware.xml  
  DESTINATION share/${PROJECT_NAME}  
)  
  
ament_package()
