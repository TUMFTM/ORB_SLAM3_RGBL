#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

xhost +local:root

XSOCK=/tmp/.X11-unix && \
docker run -it \
 -e DISPLAY=$DISPLAY \
 -v $XSOCK:$XSOCK \
 -v /LocalStorage/KITTI_odometry/dataset/:/kitti/ \
 --network=host \
 --privileged \
 rgbl:latest

 xhost -local:root
