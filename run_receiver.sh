#!/bin/bash
# Aktivuje základní ROS
source /opt/ros/jazzy/setup.bash
# Aktivuje tvůj zkompilovaný projekt
source install/setup.bash
# Spustí program (předpokládáme, že config.json je v aktuální složce)
LOG_LEVEL=$LOG_LEVEL ./build/mpc-rbt-solution/receiver_node config.json
