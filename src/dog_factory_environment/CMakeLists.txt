# Définit la version minimale de CMake.
cmake_minimum_required(VERSION 3.8)

# Déclare le package d'environnement industriel.
project(dog_factory_environment)

# Charge la construction ament pour ROS 2.
find_package(ament_cmake REQUIRED)

# Installe les mondes SDF dans le dossier share du package.
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Termine la déclaration du package.
ament_package()
