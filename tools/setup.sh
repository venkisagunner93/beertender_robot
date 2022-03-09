#!/bin/sh

TOOLS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

function beertender () {
    python3 $TOOLS_DIR/beertender_main.py $@
}

function sdk () {
    docker run -it --rm --net=host --privileged \
        -e DISPLAY=$DISPLAY \
        -e TERM \
        -e QT_X11_NO_MITSHM=1\
        -e XAUTHORITY=/tmp/.docker.xauth \
        -v /tmp/.docker.xauth:/tmp/.docker.xauth \
        -v /tmp/.X11-unix/:/tmp/.X11-unix \
        --gpus all \
        --runtime nvidia \
        --platform linux/arm64 \
        sdk-base:latest
}