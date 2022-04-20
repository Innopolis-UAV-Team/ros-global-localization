#!/bin/bash
cd "$(dirname "$0")"
source /opt/ros/$ROS_DISTRO/setup.bash
source /catkin_ws/devel/setup.bash
set -e

# set `line buffering` mode to stdout
stdbuf -o L roslaunch pc_global_localization run_pc_global_localization.launch