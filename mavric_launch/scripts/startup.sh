#!/bin/bash

if [[ $(basename $(pwd)) != "mavric_launch" ]]; then
    echo "ERROR: please run from mavric_launch directory"
    exit 1
fi

./scripts/compile_urdf.sh
./scripts/activate_lidar.sh

ros2 launch mavric_launch mavric.launch.py
