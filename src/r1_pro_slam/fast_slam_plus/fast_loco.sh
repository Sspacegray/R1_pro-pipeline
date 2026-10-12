#!/usr/bin/env bash
# fast_loco.sh — R1 Pro FAST-LIO2 + global localization + Nav2 runtime.
#
# It stops only the old r1pro_livox_loc session, keeps HDAS/Mobiman alive,
# starts one sensor input, and never publishes a navigation goal itself.
set -Eeuo pipefail

ROOT_DIR=/home/nvidia/galaxea
STACK_ROOT="$ROOT_DIR/galaxea_overlay/src/r1_pro_slam/fast_slam_plus"
ENV_SH="$STACK_ROOT/fast_loco_env.sh"
SESSION=fast_slam_plus
OLD_SESSION=r1pro_livox_loc
PCD_MAP="${1:-$STACK_ROOT/maps/virtual_machine_map.pcd}"
MAP_YAML="${2:-$ROOT_DIR/maps/bag_20260914_map.yaml}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8767}"
INITIAL_X="${INITIAL_X:-0.0}"
INITIAL_Y="${INITIAL_Y:-0.0}"
INITIAL_YAW="${INITIAL_YAW:-0.0}"
LOCALIZATION_TIMEOUT="${LOCALIZATION_TIMEOUT:-300}"

usage() {
  cat <<'EOF'
Usage:
  bash fast_loco.sh [pcd_map] [map_yaml]
  bash fast_loco.sh --stop
  bash fast_loco.sh --fallback-old

Environment:
  FOXGLOVE_PORT=8767
  INITIAL_X=0 INITIAL_Y=0 INITIAL_YAW=0
  LOCALIZATION_TIMEOUT=300

The script starts localization, map_server, Nav2, the single cmd_vel bridge,
and Foxglove. It does not publish /goal_pose. After localization is healthy,
the navigation output service is enabled automatically; an external /goal_pose
is still required before the robot can move.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
if [[ "${1:-}" == "--stop" ]]; then
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  echo "stopped tmux session: $SESSION"
  exit 0
fi
if [[ "${1:-}" == "--fallback-old" ]]; then
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  exec bash "$ROOT_DIR/start_r1pro_livox_nav.sh" "${2:-}" "${3:-$MAP_YAML}"
fi

[[ -r "$PCD_MAP" ]] || { echo "ERROR: PCD map not found: $PCD_MAP" >&2; exit 1; }
[[ -r "$MAP_YAML" ]] || { echo "ERROR: 2D map YAML not found: $MAP_YAML" >&2; exit 1; }
[[ -r "$ENV_SH" ]] || { echo "ERROR: environment file not found: $ENV_SH" >&2; exit 1; }

source "$ENV_SH"

# Keep runtime configs alive after the SSH launcher exits; tmux children read
# these files for their whole lifetime.
RUNTIME_DIR="$STACK_ROOT/runtime"
mkdir -p "$RUNTIME_DIR"
FASTLIO_CFG="$RUNTIME_DIR/fastlio.yaml"
LOCALIZER_CFG="$RUNTIME_DIR/localizer.yaml"
sed "s|^map_path:.*|map_path: $PCD_MAP|" \
  "$STACK_ROOT/install/share/localizer/config/r1pro_localizer.yaml" > "$LOCALIZER_CFG"
cp "$STACK_ROOT/install/share/fastlio2/config/r1pro_mid360.yaml" "$FASTLIO_CFG"

# Keep old base services but remove the old localization/navigation session as
# one unit, avoiding duplicate TF, Nav2 and Foxglove publishers.
old_session_was_running=false
if tmux has-session -t "$OLD_SESSION" 2>/dev/null; then
  old_session_was_running=true
fi
tmux kill-session -t "$OLD_SESSION" 2>/dev/null || true
tmux kill-session -t "$SESSION" 2>/dev/null || true
if [[ "$old_session_was_running" == true ]]; then
  pkill -TERM -f 'livox_ros_driver2_node.*__node:=livox_lidar_publisher' 2>/dev/null || true
  sleep 2
fi

cleanup_on_error() {
  echo "ERROR: fast_slam_plus startup failed; cleaning new tmux session" >&2
  tmux kill-session -t "$SESSION" 2>/dev/null || true
}
trap cleanup_on_error ERR

new_window() {
  local name="$1"
  local command="$2"
  tmux new-window -t "$SESSION" -n "$name" "bash -lc 'source \"$ENV_SH\"; $command'"
  tmux set-option -w -t "$SESSION:$name" remain-on-exit on
}

# Start a session before adding windows. Sensor is added only when no current
# publisher exists, so the script never intentionally starts two Livox drivers.
tmux new-session -d -s "$SESSION" -n preflight "bash -lc 'source \"$ENV_SH\"; exec bash'"
tmux set-option -w -t "$SESSION:preflight" remain-on-exit on

has_publisher() {
  local info
  info="$(ros2 topic info "$1" -v 2>/dev/null || true)"
  grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$info"
}

wait_for_message() {
  local topic="$1"
  local timeout_sec="${2:-30}"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if timeout 4s ros2 topic echo --once "$topic" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

if ! wait_for_message /hdas/lidar_chassis_left 5; then
  new_window sensor "exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -r __node:=livox_lidar_publisher -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame -p user_config_path:=$ROOT_DIR/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json"
fi
wait_for_message /hdas/lidar_chassis_left 60 || { echo "ERROR: no /hdas/lidar_chassis_left data" >&2; exit 1; }

new_window tf 'exec ros2 run tf2_ros static_transform_publisher 0.15711 0.26215 0.29465 0 0 0 base_link livox_frame'
new_window fastlio "exec ros2 run fastlio2 lio_node --ros-args -r __ns:=/fastlio2 -r lio_odom:=/odom -p config_path:=$FASTLIO_CFG -p use_sim_time:=False"
new_window localizer "exec ros2 run localizer localizer_node --ros-args -r __ns:=/localizer -p config_path:=$LOCALIZER_CFG -p use_sim_time:=False"
new_window model 'exec ros2 run r1_pro_navigation2 robot_description_publisher.py'
new_window foxglove "exec ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=$FOXGLOVE_PORT -p address:=0.0.0.0 --params-file $ROOT_DIR/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config/foxglove.yaml"

echo 'waiting for FAST-LIO2 odom and synchronized cloud...'
wait_for_message /odom 60 || { echo 'ERROR: /odom did not publish' >&2; exit 1; }
wait_for_message /fastlio2/body_cloud 60 || { echo 'ERROR: /fastlio2/body_cloud did not publish' >&2; exit 1; }

# Let Localizer accumulate its configured multi-scan search window before the
# first global request; a single scan is a poor global descriptor.
sleep 8
echo "publishing automatic startup /initialpose for global search at ($INITIAL_X, $INITIAL_Y, $INITIAL_YAW)"
qz="$(python3 -c 'import math,sys; print(math.sin(float(sys.argv[1])/2.0))' "$INITIAL_YAW")"
qw="$(python3 -c 'import math,sys; print(math.cos(float(sys.argv[1])/2.0))' "$INITIAL_YAW")"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: $INITIAL_X, y: $INITIAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $qz, w: $qw}}}}" >/tmp/fast_slam_plus_initialpose.log 2>&1

localization_deadline=$((SECONDS + LOCALIZATION_TIMEOUT))
localization_ok=false
while (( SECONDS < localization_deadline )); do
  if tmux capture-pane -p -t "$SESSION:localizer" -S -200 2>/dev/null \
      | grep -Eq 'global relocalization candidate=.*success=true'; then
    localization_ok=true
    break
  fi
  sleep 1
done
if [[ "$localization_ok" != true ]]; then
  echo "ERROR: global relocalization did not succeed within ${LOCALIZATION_TIMEOUT}s" >&2
  exit 1
fi

new_window map "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_YAML -p use_sim_time:=False & exec ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=True -p bond_timeout:=8.0 -p node_names:=[map_server]"
new_window nav2 "exec ros2 launch r1_pro_navigation2 nav2.launch.py use_sim_time:=False nav2_log_level:=info params_profile:=omini goal_stamp_mode:=zero goal_require_tf_ready:=True start_cmd_vel_bridge:=True nav_chassis_output_enabled:=true nav_publish_only_when_moving:=True nav_publish_zero_on_stop:=true require_localization_safe:=True"

# The bridge starts fail-closed while waiting for the first safe heartbeat. Once
# localization is healthy, explicitly arm it; no goal is sent by this script.
safe_deadline=$((SECONDS + 60))
safe_ok=false
safe_streak=0
while (( SECONDS < safe_deadline )); do
  if timeout 4s ros2 topic echo --once /navigation/localization_safe 2>/dev/null | grep -Eq 'data: true'; then
    ((safe_streak += 1))
    if (( safe_streak >= 8 )); then
      safe_ok=true
      break
    fi
  else
    safe_streak=0
  fi
  sleep 1
done
if [[ "$safe_ok" != true ]]; then
  echo 'ERROR: localization safety heartbeat did not become true' >&2
  exit 1
fi

for _ in {1..10}; do
  if ros2 service call /cmd_vel_to_twist_stamped/set_nav_output_enabled std_srvs/srv/SetBool '{data: true}' 2>/dev/null \
      | grep -Eq 'success: true'; then
    break
  fi
  sleep 1
done

trap - ERR
tmux select-window -t "$SESSION:foxglove"
echo '=============================================='
echo " FAST-LIO2 + Localizer + Nav2 started (tmux: $SESSION)"
echo "   PCD: $PCD_MAP"
echo "   2D map: $MAP_YAML"
echo "   Foxglove: ws://192.168.3.4:$FOXGLOVE_PORT"
echo "   Layout: $STACK_ROOT/config/fast_loco_layout.json"
echo '   Goal: publish geometry_msgs/msg/PoseStamped on /goal_pose'
echo '   Initial pose: /initialpose triggers global search'
echo '   Safety: localization_safe is a hard output gate'
echo '   Stop: bash fast_loco.sh --stop'
echo '=============================================='
