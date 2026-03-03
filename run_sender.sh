#!/bin/bash
# Aktivuje základní ROS
source /opt/ros/jazzy/setup.bash
# Aktivuje tvůj zkompilovaný projekt
source install/setup.bash
# Spustí program (předpokládáme, že config.json je v aktuální složce)
./build/mpc-rbt-solution/sender_node config.json
