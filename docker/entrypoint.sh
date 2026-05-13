#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /home/zerohour/xjtu_nav26/install/setup.bash ]; then
    source /home/zerohour/xjtu_nav26/install/setup.bash
fi

exec "$@"
