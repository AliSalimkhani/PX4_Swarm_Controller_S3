cmake_minimum_required(VERSION 3.8)
project(px4_swarm_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include ${YAML_CPP_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Arming node
add_executable(arming src/Arming.cpp)
ament_target_dependencies(arming rclcpp px4_msgs)

# Waypoint node
add_executable(waypoint src/ChangeWaypoint.cpp)
ament_target_dependencies(waypoint rclcpp px4_msgs)
target_link_libraries(waypoint ${YAML_CPP_LIBRARIES})

# Weighted topology neighbors node
add_executable(weighted_topology_neighbors src/SwarmControllers/WeightedTopology/WeightedTopologyNeighbors.cpp)
ament_target_dependencies(weighted_topology_neighbors rclcpp px4_msgs std_msgs custom_msgs Eigen3)

# Weighted topology controller node (regular executable)
add_executable(weighted_topology_controller src/SwarmControllers/WeightedTopology/WeightedTopologyController.cpp)
ament_target_dependencies(weighted_topology_controller rclcpp px4_msgs std_msgs custom_msgs Eigen3)

# Install C++ executables
install(TARGETS
  arming
  waypoint
  weighted_topology_neighbors
  weighted_topology_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Python support
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/simulation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
