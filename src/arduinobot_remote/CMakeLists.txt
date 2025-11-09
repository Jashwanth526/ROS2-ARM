cmake_minimum_required(VERSION 3.8)
project(arduinobot_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(class_loader REQUIRED)
find_package(arduinobot_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(task_server_simple SHARED src/task_server_simple.cpp)
target_include_directories(task_server_simple PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(task_server_simple PRIVATE "TASK_SERVER_SIMPLE_CPP_BUILDING_DLL")
ament_target_dependencies(task_server_simple
  "arduinobot_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"
  "tf2_ros"
  "tf2_geometry_msgs"
)

rclcpp_components_register_node(task_server_simple
  PLUGIN "arduinobot_remote::TaskServerSimple"
  EXECUTABLE task_server_simple_node
)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  task_server_simple
  task_server_simple_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS
  ${PROJECT_NAME}/alexa_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()