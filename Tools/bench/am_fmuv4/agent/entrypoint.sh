#!/usr/bin/env bash
set -eo pipefail

# Both paths are created by the pinned image build and do not exist on the host.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /opt/px4_msgs_ws/install/setup.bash

set -u
exec "$@"
