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

# Installe les exécutables dans le dossier lib du package.
install(TARGETS jump_controller_cpp lidar_obstacle_detector DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS dog_factory_control/autonomy_node.py dog_factory_control/keyboard_teleop.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
