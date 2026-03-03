#!/bin/bash
# launch.sh — starts MuJoCo sim only. Run gait separately via docker exec.

source /opt/ros/humble/setup.bash
source /workspace/sdk_deploy/install/setup.bash

export ROS_DOMAIN_ID=1

SIM_SCRIPT="/workspace/sdk_deploy/src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py"

echo "Starting MuJoCo simulation..."
echo "(Run gait in shell 2: docker exec -it lite3_sim_gait bash)"
echo ""

python3 "$SIM_SCRIPT"
