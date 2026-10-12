#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER="${CONTAINER:-ros-humble-dev}"
RATE="2.0"
NO_BAG=0

usage() {
  cat <<'USAGE'
Usage: ./run_global_relocalization_foxglove.sh [options]

Starts inside the existing ros-humble-dev container:
  - chassis IMU conversion
  - FAST-LIO2
  - global localizer (KISS-Matcher + Small GICP)
  - Foxglove bridge on localhost:8765
  - Galaxea bag playback

Options:
  --rate N       rosbag playback rate (default: 2.0)
  --no-bag       start ROS nodes and Foxglove bridge, but do not play the bag
  --stop         stop this stack in the container
  -h, --help     show this help

After starting, open Foxglove and connect to:
  ws://localhost:8765

Then import:
  localizer/config/foxglove_global_relocalization.json
USAGE
}

STOP=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rate) RATE="$2"; shift 2 ;;
    --no-bag) NO_BAG=1; shift ;;
    --stop) STOP=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

command -v docker >/dev/null || { echo "docker is required" >&2; exit 1; }
docker inspect "$CONTAINER" >/dev/null 2>&1 || {
  echo "Container not found: $CONTAINER" >&2
  exit 1
}

exec_in() {
  docker exec "$CONTAINER" bash -lc "$1"
}

detach_in() {
  docker exec -d "$CONTAINER" bash -lc "$1" >/dev/null
}

if (( STOP )); then
  exec_in '
    ps -eo pid=,args= | awk '\''/\/fastlio_ws\/install\/fastlio2\/lib\/fastlio2\/lio_node|\/fastlio_ws\/install\/localizer\/lib\/localizer\/localizer_node|\/tmp\/imu_rxpi\.py|foxglove_bridge|ros2 bag play \/work\/20260914/ {print $1}'\'' | while read -r pid; do
      kill -9 "$pid" 2>/dev/null || true
    done
  '
  echo "Stopped global relocalization stack in $CONTAINER."
  exit 0
fi

exec_in "test -f /fastlio_ws/install/fastlio2/lib/fastlio2/lio_node"
exec_in "test -f /fastlio_ws/install/localizer/lib/localizer/localizer_node"
exec_in "test -f /fastlio_out/chassis_backend/pgo_maps/map.pcd"
exec_in "test -f /fastlio_out/chassis_backend/pgo_maps/poses.txt"
exec_in "test -d /fastlio_out/chassis_backend/pgo_maps/patches"

# Stop only this stack, not unrelated ROS nodes in the container.
"$0" --stop >/dev/null 2>&1 || true

exec_in "mkdir -p /fastlio_out/global_relocalization_run"

# Keep the adapter in sync with the checked-in version.
docker cp "$(dirname "$0")/imu_chassis_to_fastlio.py" "$CONTAINER:/tmp/imu_rxpi.py" >/dev/null

detach_in "
  source /opt/ros/humble/setup.bash
  source /fastlio_ws/install/setup.bash
  exec python3 /tmp/imu_rxpi.py \
    > /fastlio_out/global_relocalization_run/imu_rxpi.log 2>&1
"

detach_in "
  source /opt/ros/humble/setup.bash
  source /fastlio_ws/install/setup.bash
  exec /fastlio_ws/install/fastlio2/lib/fastlio2/lio_node \
    --ros-args -r __ns:=/fastlio2 \
    -p config_path:=/fastlio_ws/src/fastlio2/config/lio_bag_chassis_rxpi.yaml \
    > /fastlio_out/global_relocalization_run/fastlio2.log 2>&1
"

detach_in "
  source /opt/ros/humble/setup.bash
  source /fastlio_ws/install/setup.bash
  exec /fastlio_ws/install/localizer/lib/localizer/localizer_node \
    --ros-args -r __ns:=/localizer \
    -p config_path:=/fastlio_ws/src/localizer/config/localizer.yaml \
    > /fastlio_out/global_relocalization_run/localizer.log 2>&1
"

detach_in "
  source /opt/ros/humble/setup.bash
  source /fastlio_ws/install/setup.bash
  exec ros2 run foxglove_bridge foxglove_bridge \
    --ros-args -p port:=8765 -p address:=0.0.0.0 \
    -p capabilities:="[clientPublish,services,connectionGraph,assets]" \
    > /fastlio_out/global_relocalization_run/foxglove_bridge.log 2>&1
"

if (( ! NO_BAG )); then
  detach_in "
    source /opt/ros/humble/setup.bash
    source /fastlio_ws/install/setup.bash
    exec ros2 bag play /work/20260914 \
      --storage sqlite3 --rate ${RATE} --read-ahead-queue-size 10000 \
      > /fastlio_out/global_relocalization_run/bag.log 2>&1
  "
fi

sleep 2

echo "Global relocalization stack started in: $CONTAINER"
echo
echo "Foxglove WebSocket: ws://localhost:8765"
echo "Layout file: $(dirname "$0")/localizer/config/foxglove_global_relocalization.json"
echo "Container logs: /fastlio_out/global_relocalization_run/"
echo
cat <<EOF
Check topics:
  docker exec -it $CONTAINER bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic list | grep -E "localizer|fastlio2"'

Trigger global relocalization manually after data is flowing:
  docker exec -it $CONTAINER bash
  source /opt/ros/humble/setup.bash; source /fastlio_ws/install/setup.bash
  ros2 service call /localizer/relocalize interface/srv/Relocalize \\
    "{pcd_path: '/fastlio_out/chassis_backend/pgo_maps/map.pcd', global_search: true, num_candidates: 128, seed: 42}"

Stop: $0 --stop
EOF
