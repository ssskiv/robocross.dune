#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash

# Source workspace if built
if [ -f /home/developer/robocross.dune/ros2_ws/install/setup.bash ]; then
    source /home/developer/robocross.dune/ros2_ws/install/setup.bash
fi

exec "$@"
exec "bash"