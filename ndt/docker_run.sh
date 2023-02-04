#!/bin/bash

docker run -it --rm \
    --name ndt_dev \
    -v "$(pwd):/ros2_ws/src/ndt" \
    ndt:dev
    # -w /ros2_ws/src/ndt \
