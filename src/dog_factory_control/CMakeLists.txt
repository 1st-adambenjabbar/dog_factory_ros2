cmake_minimum_required(VERSION 3.8)
project(dog_factory_control)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Compile le contrôleur C++ de saut.
add_executable(jump_controller_cpp src/jump_controller.cpp)
ament_target_dependencies(jump_controller_cpp rclcpp trajectory_msgs std_srvs)

# Compile le nœud C++ qui filtre et segmente les scans LiDAR.
add_executable(lidar_obstacle_detector src/lidar_obstacle_detector.cpp)
ament_target_dependencies(
  lidar_obstacle_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

# Compile le nœud C++ qui fusionne le LiDAR et le flux caméra.
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
ament_target_dependencies(
  sensor_fusion_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Installe les trois exécutables C++ du package.
install(TARGETS jump_controller_cpp lidar_obstacle_detector sensor_fusion_node DESTINATION lib/${PROJECT_NAME})

# Active l'intégration pytest lorsqu'elle est disponible dans ROS 2.
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(lidar_perception_pytest tests/test_lidar_obstacle_detector.py)
endif()
# Installe les nœuds Python exécutables par ros2 run.
install(
  PROGRAMS
    dog_factory_control/autonomy_node.py
    dog_factory_control/keyboard_teleop.py
    dog_factory_control/jump_state_machine.py
    dog_factory_control/evaluate_jump_policy.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
