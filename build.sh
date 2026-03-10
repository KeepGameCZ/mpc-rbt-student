#!/bin/bash
# Smaže staré buildy pro jistotu
rm -rf build install log
# Aktivuje ROS (uprav jazzy/humble podle své verze)
source /opt/ros/jazzy/setup.bash
# Spustí kompilaci
colcon build
