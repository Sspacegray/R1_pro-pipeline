#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER="${CONTAINER:-ros-humble-dev}"
DOMAIN="${ROS_DOMAIN_ID:-0}"
RATE="0.3"
BAG="/work/20260914"
PCD="/fastlio_out/chassis_backend/pgo_maps/map.pcd"
MAP_YAML="/work/offline_test/output/bag_20260914_map.yaml"
KEEP_RUNNING="${FASTLIO_NAV_KEEP:-0}"
STOP=0
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$ROOT/.." && pwd)"
TMP="/tmp/fastlio_origin_nav_${DOMAIN}"
OUT="$REPO_ROOT/test_results/origin_nav_integration"

usage() {
  cat <<'USAGE'
Usage: ./run_fastlio_origin_nav_offline.sh [options]

Runs the offline same-domain chain in the existing ros-humble-dev container:
  bag -> chassis IMU adapter -> FAST-LIO2 (odom->body)
      -> Localizer (map->odom) -> map/scan/pose bridge
      -> origin_nav -> /cmd_vel (no real hardware bridge)

Options:
  --rate N       bag playback rate (default: 0.3)
  --bag PATH     bag path inside the container
  --pcd PATH     PCD path inside the container
  --map-yaml PATH  Nav occupancy-map YAML inside the container
  --keep         leave the offline stack running for inspection
  --stop         stop only this domain/work directory
  -h, --help     show this help

The default map YAML is the existing processed 2-D map. If it is empty, the
bridge projects the PCD into an OccupancyGrid instead.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rate) RATE="$2"; shift 2 ;;
    --bag) BAG="$2"; shift 2 ;;
    --pcd) PCD="$2"; shift 2 ;;
    --map-yaml) MAP_YAML="$2"; shift 2 ;;
    --keep) KEEP_RUNNING=1; shift ;;
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

exec_in() { docker exec "$CONTAINER" bash -lc "$1"; }
detach_in() { docker exec -d "$CONTAINER" bash -lc "$1" >/dev/null; }

stop_stack() {
  exec_in "
    if test -d '$TMP/pids'; then
      for file in '$TMP'/pids/*; do
        test -f \"\$file\" || continue
        kill -TERM \"\$(cat \"\$file\")\" 2>/dev/null || true
      done
    fi
    ps -eo pid=,args= | awk -v me=\$\$ -v parent=\$PPID -v pattern='$TMP' \
      'index(\$0, pattern) && \$1 != me && \$1 != parent {print \$1}' | \
      xargs -r kill -TERM 2>/dev/null || true
    sleep 1
    if test -d '$TMP/pids'; then
      for file in '$TMP'/pids/*; do
        test -f \"\$file\" || continue
        kill -KILL \"\$(cat \"\$file\")\" 2>/dev/null || true
      done
    fi
    ps -eo pid=,args= | awk -v me=\$\$ -v parent=\$PPID -v pattern='$TMP' \
      'index(\$0, pattern) && \$1 != me && \$1 != parent {print \$1}' | \
      xargs -r kill -KILL 2>/dev/null || true
    rm -f '$TMP'/pids/* 2>/dev/null || true
  "
}

if (( STOP )); then
  stop_stack
  echo "Stopped FAST-LIO2/origin_nav stack for ROS_DOMAIN_ID=$DOMAIN."
  exit 0
fi

mkdir -p "$OUT"
stop_stack || true
exec_in "rm -rf '$TMP'; mkdir -p '$TMP' '$TMP/nav_ws/src' '$TMP/run' '$TMP/pids'"

# Copy only the source/config needed for this isolated run.
docker cp "$ROOT/scripts/fastlio_nav_bridge.py" "$CONTAINER:$TMP/fastlio_nav_bridge.py"
docker cp "$ROOT/tools/fastlio_origin_nav_smoke.py" "$CONTAINER:$TMP/fastlio_origin_nav_smoke.py"
docker cp "$ROOT/imu_chassis_to_fastlio.py" "$CONTAINER:$TMP/imu_chassis_to_fastlio.py"
docker cp "$ROOT/fastlio2/config/lio_bag_chassis_rxpi_nav.yaml" "$CONTAINER:$TMP/lio.yaml"
docker cp "$ROOT/localizer/config/localizer.yaml" "$CONTAINER:$TMP/localizer.yaml"
docker cp "$REPO_ROOT/origin_nav/src" "$CONTAINER:$TMP/nav_ws/src"

exec_in "
  source /opt/ros/humble/setup.bash
  colcon build --base-paths '$TMP/nav_ws/src' --build-base '$TMP/nav_build' \
    --install-base '$TMP/nav_install' --packages-up-to navigation \
    --event-handlers console_direct+ --executor sequential > '$TMP/nav_build.log' 2>&1
"

exec_in "test -f '$BAG/metadata.yaml'"
exec_in "test -f '$PCD'"
if [[ -n "$MAP_YAML" ]]; then exec_in "test -f '$MAP_YAML'"; fi

COMMON="source /opt/ros/humble/setup.bash; export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=$DOMAIN"

# /clock is required: bag timestamps are historical and origin_nav has age gates.
detach_in "$COMMON; echo \$\$ > '$TMP/pids/bag'; exec -a '$TMP/bag_player' ros2 bag play '$BAG' --storage sqlite3 --rate '$RATE' --loop --clock --read-ahead-queue-size 10000 > '$TMP/run/bag.log' 2>&1"
sleep 2
detach_in "$COMMON; echo \$\$ > '$TMP/pids/imu'; exec python3 '$TMP/imu_chassis_to_fastlio.py' --ros-args -p use_sim_time:=true > '$TMP/run/imu.log' 2>&1"
detach_in "$COMMON; echo \$\$ > '$TMP/pids/fastlio2'; source /fastlio_ws/install/setup.bash; exec /fastlio_ws/install/fastlio2/lib/fastlio2/lio_node --ros-args -r __ns:=/fastlio2 -p config_path:='$TMP/lio.yaml' -p use_sim_time:=true > '$TMP/run/fastlio2.log' 2>&1"
detach_in "$COMMON; echo \$\$ > '$TMP/pids/localizer'; source /fastlio_ws/install/setup.bash; exec /fastlio_ws/install/localizer/lib/localizer/localizer_node --ros-args -r __ns:=/localizer -p config_path:='$TMP/localizer.yaml' -p use_sim_time:=true > '$TMP/run/localizer.log' 2>&1"
detach_in "$COMMON; echo \$\$ > '$TMP/pids/bridge'; exec python3 '$TMP/fastlio_nav_bridge.py' --ros-args -p pcd_path:='$PCD' -p map_yaml:='$MAP_YAML' -p use_sim_time:=true > '$TMP/run/bridge.log' 2>&1"
detach_in "$COMMON; echo \$\$ > '$TMP/pids/tf_base'; exec /opt/ros/humble/lib/tf2_ros/static_transform_publisher 0 0 0 0 0 0 1 body base_footprint > '$TMP/run/tf_base.log' 2>&1"
detach_in "$COMMON; echo \$\$ > '$TMP/pids/tf_livox'; exec /opt/ros/humble/lib/tf2_ros/static_transform_publisher 0 0 0 0 0 0 1 body livox_frame > '$TMP/run/tf_livox.log' 2>&1"

sleep 20
exec_in "$COMMON; source /fastlio_ws/install/setup.bash; timeout 120 ros2 service call /localizer/relocalize interface/srv/Relocalize \"{pcd_path: '$PCD', global_search: true, num_candidates: 128, seed: 42}\" > '$TMP/run/relocalize.log' 2>&1"

detach_in "$COMMON; echo \$\$ > '$TMP/pids/nav_launch'; source '$TMP/nav_install/setup.bash'; exec ros2 launch navigation origin_nav_irsim.launch.py robot_name:=r1 use_sim_time:=true odom_topic:=/fastlio2/lio_odom scan_topic:=/scan max_tf_age_sec:=0.5 max_odom_age_sec:=0.5 > '$TMP/run/nav.log' 2>&1"
sleep 10

set +e
exec_in "$COMMON; export FASTLIO_NAV_REPORT='$TMP/smoke_report.json'; python3 '$TMP/fastlio_origin_nav_smoke.py' --ros-args -p use_sim_time:=true > '$TMP/run/smoke.log' 2>&1"
SMOKE_RC=$?
set -e

# Always copy evidence before optional cleanup.
for file in nav_build.log smoke_report.json; do
  docker cp "$CONTAINER:$TMP/$file" "$OUT/$file" 2>/dev/null || true
done
for file in bag.log imu.log fastlio2.log localizer.log bridge.log tf_base.log tf_livox.log relocalize.log nav.log smoke.log; do
  docker cp "$CONTAINER:$TMP/run/$file" "$OUT/$file" 2>/dev/null || true
done

if (( ! KEEP_RUNNING )); then stop_stack || true; fi
if (( SMOKE_RC != 0 )); then
  echo "FAIL: real FAST-LIO2/Localizer/origin_nav smoke; evidence: $OUT" >&2
  exit "$SMOKE_RC"
fi

echo "PASS: real bag -> FAST-LIO2 -> Localizer -> origin_nav smoke"
echo "Evidence: $OUT"
echo "ROS_DOMAIN_ID=$DOMAIN; map source=${MAP_YAML:-PCD projection}; keep=$KEEP_RUNNING"
