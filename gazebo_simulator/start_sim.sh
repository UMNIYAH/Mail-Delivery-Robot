#!/bin/bash

set -e

WORKSPACE=~/testing_ws
cd "$WORKSPACE"

echo "Sourcing workspace and ROS2..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting Gazebo"
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py world_path:=$HOME/.gazebo/worlds/demo_video.world spawn_beacons:=true