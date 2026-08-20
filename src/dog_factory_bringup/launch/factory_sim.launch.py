# Importe l'objet qui décrit l'ensemble du lancement ROS 2.
from launch import LaunchDescription

# Importe l'action permettant d'inclure un autre fichier launch.
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

# Importe la source utilisée pour charger un launch Python externe.
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Importe la condition d'activation optionnelle de Nav2.
from launch.conditions import IfCondition

# Importe les substitutions évaluées au moment du lancement.
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

# Importe l'action Node qui démarre un exécutable ROS 2.
from launch_ros.actions import Node

# Importe la substitution qui localise le dossier share d'un package.
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Construit le lancement complet de la simulation d'usine."""

    # Localise le package contenant le Xacro du robot.
    description_package = FindPackageShare('dog_robot_description')

    # Localise le package contenant le monde Gazebo.
    gazebo_package = FindPackageShare('dog_factory_environment')

    # Localise le package contenant les paramètres de bringup.
    bringup_package = FindPackageShare('dog_factory_bringup')

    # Construit le chemin vers le monde SDF de l'usine.
    world_file = PathJoinSubstitution([gazebo_package, 'worlds', 'factory.world'])

    # Construit le chemin vers le modèle Xacro du robot.
    xacro_file = PathJoinSubstitution([description_package, 'urdf', 'dog_robot_core.xacro'])

    # Retourne la liste des actions exécutées par ros2 launch.
    return LaunchDescription([
        # Permet de choisir si Gazebo doit ouvrir son interface graphique.
        DeclareLaunchArgument('gui', default_value='true'),

        # Permet d'activer Nav2 depuis le lancement global.
        DeclareLaunchArgument('navigation', default_value='false'),

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

        # Démarre la fusion C++ entre les données du LiDAR et de la caméra.
        Node(
            package='dog_factory_control',
            executable='sensor_fusion_node',
            parameters=[{
                'image_topic': '/image_raw',
                'front_angle': 0.45,
                'obstacle_distance': 2.0,
                'camera_timeout': 0.5,
            }],
            output='screen',
        ),

        # Démarre la machine à états Python pour les sauts par-dessus obstacles.
        Node(
            package='dog_factory_control',
            executable='jump_state_machine',
            parameters=[{
                'auto_jump': False,
                'jump_trigger_distance': 0.75,
            }],
            output='screen',
        ),

        # Démarre le contrôleur C++ historique de saut pour compatibilité.
        Node(
            package='dog_factory_control',
            executable='jump_controller_cpp',
            output='screen',
        ),

        # Démarre la localisation et la navigation Nav2 si navigation=true.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('dog_factory_navigation'),
                    'launch',
                    'navigation.launch.py',
                ])
            ),
            condition=IfCondition(LaunchConfiguration('navigation')),
        ),
    ])
