cmake_minimum_required(VERSION 3.5)
project(simulate)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(simulate src/manualArea.cpp)

# Link dependencies to the executable
ament_target_dependencies(simulate
  rclcpp
  std_msgs
)

# Install the executable
install(TARGETS
  simulate
  DESTINATION lib/${PROJECT_NAME})

# Install launch 
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
