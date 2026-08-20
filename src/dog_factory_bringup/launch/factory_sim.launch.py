# Importe l'objet qui décrit l'ensemble du lancement ROS 2.
from launch import LaunchDescription

# Importe l'action permettant d'inclure un autre fichier launch.
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

# Importe la source utilisée pour charger un launch Python externe.
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Importe les substitutions évaluées au moment du lancement.
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

# Importe l'action Node qui démarre un exécutable ROS 2.
from launch_ros.actions import Node

# Importe la substitution qui localise le dossier share d'un package.
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Construit le lancement complet de la simulation d'usine."""

    # Localise le package contenant le Xacro du robot.
    description_package = FindPackageShare('dog_factory_description')

    # Localise le package contenant le monde Gazebo.
    gazebo_package = FindPackageShare('dog_factory_gazebo')

    # Localise le package contenant les paramètres de bringup.
    bringup_package = FindPackageShare('dog_factory_bringup')

    # Construit le chemin vers le monde SDF de l'usine.
    world_file = PathJoinSubstitution([gazebo_package, 'worlds', 'factory.world'])

    # Construit le chemin vers le modèle Xacro du robot.
    xacro_file = PathJoinSubstitution([description_package, 'urdf', 'dog.urdf.xacro'])

    # Retourne la liste des actions exécutées par ros2 launch.
    return LaunchDescription([
        # Permet de choisir si Gazebo doit ouvrir son interface graphique.
        DeclareLaunchArgument('gui', default_value='true'),

        # Inclut le launch officiel gazebo_ros avec le monde usine sélectionné.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py',
                ])
            ),
            launch_arguments={
                'world': world_file,
                'gui': LaunchConfiguration('gui'),
            }.items(),
        ),

        # Convertit le Xacro en URDF et publie les transformations du robot.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file]),
            }],
            output='screen',
        ),

        # Insère l'entité décrite par robot_description dans Gazebo.
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'factory_dog',
                '-x', '-4.0',
                '-y', '0.0',
                '-z', '0.8',
            ],
            output='screen',
        ),

        # Démarre l'autonomie Python qui consomme le topic lidar /scan.
        Node(
            package='dog_factory_control',
            executable='autonomy_node',
            parameters=[
                PathJoinSubstitution([bringup_package, 'config', 'controllers.yaml']),
            ],
            output='screen',
        ),

        # Démarre le détecteur C++ qui consomme /scan et publie les obstacles.
        Node(
            package='dog_factory_control',
            executable='lidar_obstacle_detector',
            parameters=[{
                'min_range': 0.12,
                'max_range': 12.0,
                'cluster_distance': 0.35,
                'min_cluster_points': 3,
                'front_danger_distance': 1.0,
            }],
            output='screen',
        ),

        # Démarre le nœud C++ qui fournit le service /dog/jump.
        Node(
            package='dog_factory_control',
            executable='jump_controller_cpp',
            output='screen',
        ),
    ])
