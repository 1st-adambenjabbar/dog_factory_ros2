from setuptools import setup
package_name = 'dog_factory_control'
setup(name=package_name, version='0.1.0', packages=[package_name], data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml'])], install_requires=['setuptools'], entry_points={'console_scripts':['autonomy_node = dog_factory_control.autonomy_node:main','keyboard_teleop = dog_factory_control.keyboard_teleop:main']})
