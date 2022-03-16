#!/bin/sh

TOOLS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE=${HOME}/beertender_robot

function beertender () {
    python3 $TOOLS_DIR/beertender_main.py $@
}

function sdk () {
    docker run --rm \
        --runtime=nvidia \
        -e "DISPLAY=${DISPLAY:-:0.0}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v $HOME/beertender_robot:/home/user/beertender_robot:rw \
        --device /dev/input/js0 \
        --security-opt apparmor:unconfined \
        --net=host \
        --gpus all \
        -it sdk-base:latest
}