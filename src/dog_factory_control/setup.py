# Importe la fonction setuptools qui décrit l'installation Python.
from setuptools import setup

# Définit le nom utilisé par ROS 2 pour ce package.
package_name = 'dog_factory_control'

# Déclare les modules Python et leurs commandes `ros2 run`.
setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'autonomy_node = dog_factory_control.autonomy_node:main',
            'keyboard_teleop = dog_factory_control.keyboard_teleop:main',
            'jump_state_machine = dog_factory_control.jump_state_machine:main',
            'evaluate_jump_policy = dog_factory_control.evaluate_jump_policy:main',
        ],
    },
)
