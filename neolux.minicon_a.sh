#!/bin/bash

cd /root/dev_ws/minicon_a || exit 1

source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source ./install/setup.bash

ros2 launch april_tag_tracker minicon_a.launch.py