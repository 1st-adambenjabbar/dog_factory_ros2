#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/../install/setup.bash"
echo 'Topics: ros2 topic echo /scan | ros2 topic echo /odom'
echo 'Trigger jump: ros2 service call /dog/jump std_srvs/srv/Trigger {}'
echo 'Stop: ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"'
