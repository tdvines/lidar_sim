#!/bin/bash
# Add all installed boot models to GAZEBO_MODEL_PATH
for prefix in $AMENT_PREFIX_PATH; do
    if [ -d "$prefix/share/boot/models" ]; then
        export GAZEBO_MODEL_PATH="$prefix/share/boot/models:${GAZEBO_MODEL_PATH}"
    fi
done
export GAZEBO_MODEL_PATH
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
echo "GAZEBO_MODEL_PATH: 

