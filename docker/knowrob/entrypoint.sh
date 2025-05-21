#!/bin/bash

source /opt/ros/overlay_ws/devel/setup.bash
roslaunch knowrob_designator knowrob.launch

set -e

exec "$@"