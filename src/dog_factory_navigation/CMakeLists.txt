# Définit la version minimale de CMake.
cmake_minimum_required(VERSION 3.8)
# Déclare le package de navigation.
project(dog_factory_navigation)
# Charge l'outil de construction ROS 2.
find_package(ament_cmake REQUIRED)
# Installe les launches, paramètres, cartes et configurations RViz.
install(DIRECTORY launch config maps rviz DESTINATION share/${PROJECT_NAME})
# Finalise le package ament.
ament_package()
