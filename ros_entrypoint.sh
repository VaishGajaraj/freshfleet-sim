#!/bin/bash
set -e
# source ROS and workspace
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
exec "$@" 