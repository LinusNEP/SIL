#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/sil_ws/devel/setup.bash
exec "$@"

