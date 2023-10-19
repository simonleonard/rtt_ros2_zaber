cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_zaber)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtt_ros2 REQUIRED)
find_package(rtt_ros2_node REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_reproduce_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  eigen-3.4.0
)

orocos_component(rtt_ros2_zaber_base src/rtt_ros2_zaber_base.cpp)
target_link_libraries(rtt_ros2_zaber_base
  rtt_ros2::rtt_ros2
  rtt_ros2_node::rtt_ros2_node
  zml
)
ament_target_dependencies(rtt_ros2_zaber_base
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  control_reproduce_interfaces
)

orocos_component(rtt_ros2_zaber_auto_insertion 
  src/rtt_ros2_zaber_auto_insertion.cpp 
)
target_link_libraries(rtt_ros2_zaber_auto_insertion
  rtt_ros2_zaber_base
)

orocos_component(rtt_ros2_zaber_control_reproduce
  src/rtt_ros2_zaber_control_reproduce.cpp
  src/sg_filter.cpp
  src/traj_collector.cpp
)
target_link_libraries(rtt_ros2_zaber_control_reproduce
  rtt_ros2_zaber_base
)
ament_target_dependencies(rtt_ros2_zaber_control_reproduce
  std_srvs
  tf2_geometry_msgs
  sensor_msgs
)

orocos_generate_package()

ament_package()
