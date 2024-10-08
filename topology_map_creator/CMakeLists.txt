cmake_minimum_required(VERSION 3.5)
project(topology_map_creator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

# Declare the message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Area2D.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Create executable
add_executable(topology_map_node src/TopologyMapNode.cpp)

# Link dependencies to the executable
ament_target_dependencies(topology_map_node
  rclcpp
  std_msgs
  geometry_msgs
  rosidl_default_runtime
)

# Link message type support libraries
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(topology_map_node "${cpp_typesupport_target}")

# Install the executable
install(TARGETS
  topology_map_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
