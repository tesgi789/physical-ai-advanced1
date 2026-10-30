#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash

cd /home/ubuntu/pai_project

echo "Reset Gazebo..."
ros2 service call /reset_world std_srvs/srv/Empty "{}"

echo "Starting in 5 seconds..."
sleep 5

python3 ai_avoid_robot_long.py
