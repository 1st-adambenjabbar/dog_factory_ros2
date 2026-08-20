# Exige une version moderne minimale de CMake.
cmake_minimum_required(VERSION 3.8)

# Déclare le nom du package de description robot.
project(dog_robot_description)

# Charge les macros de construction ROS 2.
find_package(ament_cmake REQUIRED)

# Installe toutes les descriptions Xacro et ressources sous share/.
install(
  DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

# Finalise l'export du package ament.
ament_package()
