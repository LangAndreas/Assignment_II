#!/bin/bash
mkdir -p ~/ros2_ws/src
cp -r ../Assignment_II-main ~/ros2_ws/src

cd ~/ros2_ws

mv ./src/Assignment_II-main ./src/turtlebot_control

colcon build --symlink-install

source ~/ros2_ws/install/setup.bash
source /opt/ros/humble/setup.bash

ros2 run turtlesim turtlesim_node & ros2 run turtlebot_control moveNode
