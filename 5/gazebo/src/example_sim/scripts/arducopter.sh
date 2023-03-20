#!/bin/bash

if [[ -z "$1" || -z "$2" ]]
then
    echo "Usage: ./arducopter.sh <arducopter instance> <temp directory> <param_file> <location>"
    exit 1
else
    instance=$1
    temp_directory=$2
fi

cd /workspace
mkdir -p temp/$temp_directory
./ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -I $instance --use-dir temp/$temp_directory