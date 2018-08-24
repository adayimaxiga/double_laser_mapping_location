#!/bin/bash
echo "I have a dream"

export TURTLEBOT3_MODEL=waffle
source /opt/ros/melodic/setup.bash
source ~/ros1_ws/devel_isolated/setup.bash
roslaunch launch_environment.launch

