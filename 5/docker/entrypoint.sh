#!/bin/bash
set -e

source /usr/share/gazebo-11/setup.sh
source /opt/ros/humble/setup.bash
source /workspace/gazebo/install/setup.bash

export GAZEBO_MODEL_PATH=/workspace/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=/workspace/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=/workspace/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=/workspace/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}

exec "$@"
