#!/bin/bash

docker run -it --rm \
    --name ndt_locator \
    -v "$(pwd):/ros2_ws/src/ndt" \
    -v "/Users/eviemai/Data:/ros2_ws/src/ndt/maps:ro" \
    ndt:dev
