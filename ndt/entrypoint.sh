#!/bin/bash
set -e

echo "Executing the Entrypoint..."
source /opt/ros/humble/setup.sh

# Source the main workspace, if built
if [ -f /ros2_ws/install/setup.bash ]
then
  echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
  source /ros2_ws/install/setup.bash
  echo "Sourced ROS2 workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"