cmake_minimum_required(VERSION 3.5)
project(drone_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavlink REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include Cpp "include" directory
# include_directories(include)
include_directories(
  # ${mavlink_INCLUDE_DIRS}
)

# Create Cpp executable
add_executable(vision_bridge src/vision_bridge.cpp)
ament_target_dependencies(vision_bridge
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  mavros
  mavros_msgs
  mavlink
)

# Install Cpp executables
install(TARGETS vision_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  drone_control/example_missions/simple_mission.py
  drone_control/example_missions/camera_test_mission.py
  drone_control/example_missions/simple_duo_mission.py
  drone_control/fielded_missions/move_test_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


