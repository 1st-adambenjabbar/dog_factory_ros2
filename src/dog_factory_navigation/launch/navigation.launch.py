# Décrit la liste des actions du launch ROS 2.
from launch import LaunchDescription

# Permet d'inclure les launch officiels Nav2.
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Construit les chemins vers les fichiers installés.
from launch.substitutions import PathJoinSubstitution

# Localise les packages ROS 2 dans l'espace de partage.
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Démarre la localisation AMCL et la navigation Nav2."""

    # Localise le package de navigation du projet.
    navigation_package = FindPackageShare('dog_factory_navigation')

    # Localise le package officiel nav2_bringup.
    nav2_package = FindPackageShare('nav2_bringup')

    # Construit le chemin vers les paramètres communs Nav2.
    params_file = PathJoinSubstitution([
        navigation_package,
        'config',
        'nav2_params.yaml',
    ])

    # Construit le chemin vers la carte d'occupation.
    map_file = PathJoinSubstitution([
        navigation_package,
        'maps',
        'factory.yaml',
    ])

    # Retourne deux inclusions officielles : localisation puis navigation.
    return LaunchDescription([
        # Lance map_server et AMCL pour fournir map -> odom.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    nav2_package,
                    'launch',
                    'localization_launch.py',
                ])
            ),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'True',
                'autostart': 'True',
            }.items(),
        ),
        # Lance planner_server, controller_server et BT navigator.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    nav2_package,
                    'launch',
                    'navigation_launch.py',
                ])
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'True',
                'autostart': 'True',
            }.items(),
        ),
    ])
