#!/usr/bin/env bash
set -euo pipefail

image=${1:-am-px4-jazzy-xrce:fw02}
repo=${2:-$(git rev-parse --show-toplevel)}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

set -a
source "$script_dir/pins.env"
set +a

docker image inspect "$image" >/dev/null
test "$(docker image inspect --format '{{ index .Config.Labels "org.opencontainers.image.base.name" }}' "$image")" = "$ROS_IMAGE"
test "$(docker image inspect --format '{{ index .Config.Labels "io.lecar.xrce-agent.commit" }}' "$image")" = "$XRCE_AGENT_COMMIT"
test "$(docker image inspect --format '{{ index .Config.Labels "io.lecar.fastcdr.commit" }}' "$image")" = "$FASTCDR_COMMIT"
test "$(docker image inspect --format '{{ index .Config.Labels "io.lecar.fastdds.commit" }}' "$image")" = "$FASTDDS_COMMIT"
test "$(docker image inspect --format '{{ index .Config.Labels "io.lecar.px4-msgs.commit" }}' "$image")" = "$PX4_MSGS_COMMIT"
test "$(docker image inspect --format '{{ index .Config.Labels "io.lecar.authority" }}' "$image")" = subscribe-only-lab02a
docker run --rm "$image" bash -lc \
  'MicroXRCEAgent --help >/tmp/agent-help 2>&1 || true; grep -q "Available arguments" /tmp/agent-help'
docker run --rm "$image" python3 /opt/am_fw02/shadow_monitor.py --help >/dev/null

docker run --rm -v "$repo:/px4:ro" "$image" bash -lc '
set -e
test "$(git -C /opt/px4_msgs_ws/src/px4_msgs rev-parse HEAD)" = \
  598c7aad7b2386f9406ebd2a2f841619fddc3c78
python3 -c "from px4_msgs.msg import VehicleStatus, InputRc"
for name in BatteryStatus FailsafeFlags InputRc SensorCombined TimesyncStatus \
  VehicleAttitude VehicleLocalPosition VehicleOdometry VehicleStatus; do
  if test -f "/px4/msg/versioned/${name}.msg"; then
    source_message="/px4/msg/versioned/${name}.msg"
  else
    source_message="/px4/msg/${name}.msg"
  fi
  cmp "$source_message" "/opt/px4_msgs_ws/src/px4_msgs/msg/${name}.msg"
done
'

echo "PASS: pinned Jazzy Agent image and observed PX4 v1.18 messages"
