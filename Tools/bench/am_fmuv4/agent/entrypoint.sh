#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/px4_msgs_ws/install/setup.bash

set -u
exec "$@"
