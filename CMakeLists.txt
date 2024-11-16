cmake_minimum_required(VERSION 3.5)
project(ackermann_odom)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Float32Stamped.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Float32Stamped.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)

add_executable(ackermann_odom_node src/ackermann_odom.cpp)
add_executable(encoder_node src/encoder_node.cpp)
add_executable(dummy_steering_publisher src/dummy_steering_publisher.cpp)

ament_target_dependencies(ackermann_odom_node rclcpp sensor_msgs std_msgs nav_msgs message_filters)
ament_target_dependencies(encoder_node rclcpp sensor_msgs)
ament_target_dependencies(dummy_steering_publisher rclcpp std_msgs)


rosidl_target_interfaces(ackermann_odom_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(ackermann_odom_node "${cpp_typesupport_target}")

install(TARGETS
  ackermann_odom_node
  encoder_node
  dummy_steering_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_export_dependencies(rosidl_default_runtime)
ament_package()