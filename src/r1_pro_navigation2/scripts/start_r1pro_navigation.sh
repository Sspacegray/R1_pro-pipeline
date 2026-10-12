#!/usr/bin/env bash

# Start one navigation stack, wait for active/healthy AMCL, then restore EHI.
set -Ee -o pipefail

navigation_started_by_script=false
navigation_pgid=''
ehi_started_by_script=false

handle_error() {
  local rc=$?
  local failed_line=$1
  trap - ERR
  set +e
  printf 'ERROR: navigation startup stopped at line %s (rc=%s)\n' \
    "$failed_line" "$rc" >&2
  if [ "$navigation_started_by_script" = true ]; then
    echo 'Cleaning the partial navigation stack created by this run...' >&2
    tmux kill-session -t navigation 2>/dev/null
    if [ -n "$navigation_pgid" ]; then
      for _attempt in $(seq 1 15); do
        if ! ps -eo pgid= | awk -v pgid="$navigation_pgid" \
            '$1 == pgid {found=1} END {exit !found}'; then
          break
        fi
        sleep 1
      done
      leftovers=$(ps -eo pid=,pgid=,args= | \
        awk -v pgid="$navigation_pgid" '$2 == pgid {print}')
      if [ -n "$leftovers" ]; then
        echo 'WARNING: partial navigation processes remain:' >&2
        printf '%s\n' "$leftovers" >&2
      fi
    fi
  fi
  if [ "$ehi_started_by_script" = true ]; then
    sudo systemctl stop ehi-gateway.service >/dev/null 2>&1
  fi
  exit "$rc"
}

trap 'handle_error "$LINENO"' ERR

DDS_CONFIG=/home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config
MAP_YAML=/home/nvidia/galaxea/maps/bag_20260914_map.yaml

test -r "$DDS_CONFIG/source_fastdds_simple.sh"
test -r "$MAP_YAML"

source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/setup.bash
source "$DDS_CONFIG/source_fastdds_simple.sh"

for required_command in grep pgrep python3 sed sudo systemctl timeout tmux; do
  command -v "$required_command" >/dev/null || {
    echo "ERROR: required command not found: $required_command" >&2
    exit 1
  }
done

# Validate authority before creating a navigation stack that cannot restore EHI.
sudo -v

for expected_session in hdas mobiman ros_discovery tools; do
  if ! tmux has-session -t "$expected_session" 2>/dev/null; then
    echo "ERROR: required base tmux session is missing: $expected_session" >&2
    exit 1
  fi
done

if tmux has-session -t navigation 2>/dev/null; then
  echo 'ERROR: navigation tmux already exists; refusing a duplicate stack' >&2
  echo 'Use README section 7.2 when a navigation-only restart is required.' >&2
  exit 1
fi

mapfile -t old_navigation_pids < <(pgrep -f \
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch r1_pro_navigation2 body_attach\.launch\.py( |$)' || true)
mapfile -t old_amcl_pids < <(pgrep -f \
  '^/opt/ros/humble/lib/nav2_amcl/amcl .*__node:=amcl( |$)' || true)
if [ "${#old_navigation_pids[@]}" -gt 0 ] || [ "${#old_amcl_pids[@]}" -gt 0 ]; then
  echo 'ERROR: orphan navigation/AMCL process found; refusing a duplicate stack' >&2
  printf 'navigation=%s\n' "${old_navigation_pids[*]:-(none)}" >&2
  printf 'amcl=%s\n' "${old_amcl_pids[*]:-(none)}" >&2
  exit 1
fi

tmux new-session -d -s navigation "bash -lc '
export R1PRO_FASTDDS_DISCOVERY_MODE=simple
source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config/source_fastdds_simple.sh
exec ros2 launch r1_pro_navigation2 body_attach.launch.py \\
  map_yaml:=/home/nvidia/galaxea/maps/bag_20260914_map.yaml
'"
navigation_started_by_script=true
navigation_pane_pid=$(tmux list-panes -t navigation -F '#{pane_pid}')
navigation_pgid=$(ps -o pgid= -p "$navigation_pane_pid" | tr -d ' ')

wait_for_unique_process() {
  local pattern=$1
  local label=$2
  local count
  for _attempt in $(seq 1 60); do
    count=$(pgrep -fc "$pattern" || true)
    if [ "$count" -eq 1 ]; then
      return 0
    fi
    if [ "$count" -gt 1 ]; then
      echo "ERROR: duplicate $label processes ($count)" >&2
      return 1
    fi
    sleep 1
  done
  echo "ERROR: $label did not start" >&2
  return 1
}

wait_for_unique_process \
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch r1_pro_navigation2 body_attach\.launch\.py( |$)' \
  navigation-launch
wait_for_unique_process \
  '^/opt/ros/humble/lib/nav2_amcl/amcl .*__node:=amcl( |$)' \
  AMCL
wait_for_unique_process \
  '/r1_pro_odom/lib/r1_pro_odom/localization_to_odom\.py .*__node:=localization_to_odom( |$)' \
  localization_to_odom
wait_for_unique_process \
  '^/home/nvidia/galaxea/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node( |$)' \
  livox_ros_driver2

amcl_state=''
for _attempt in $(seq 1 30); do
  if amcl_state=$(timeout --signal=INT --kill-after=2 6 \
      ros2 lifecycle get /amcl --no-daemon 2>/dev/null) && \
     grep -Eq '^active \[3\]$' <<<"$amcl_state"; then
    break
  fi
  sleep 1
done
if ! grep -Eq '^active \[3\]$' <<<"$amcl_state"; then
  echo 'ERROR: AMCL did not enter lifecycle active' >&2
  printf '%s\n' "$amcl_state" >&2
  exit 1
fi

health_json=$(timeout --signal=INT --kill-after=2 90 ros2 topic echo \
  /navigation/amcl_health std_msgs/msg/String --no-daemon --once --field data \
  --full-length --no-lost-messages --qos-reliability reliable \
  --qos-durability transient_local \
  --filter "'\"state\":\"healthy\"' in m.data and '\"safe\":true' in m.data")
health_json=$(printf '%s\n' "$health_json" | sed '/^---$/d')
printf '%s\n' "$health_json" | python3 -c \
  'import json,sys; d=json.load(sys.stdin); assert d["state"] == "healthy" and d["safe"] is True and d["lifecycle_state"] == "active"'

sudo systemctl start ehi-gateway.service
ehi_started_by_script=true
if ! systemctl is-active --quiet ehi-gateway.service; then
  echo 'ERROR: ehi-gateway.service did not start' >&2
  exit 1
fi

navigation_pid=$(pgrep -f \
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch r1_pro_navigation2 body_attach\.launch\.py( |$)')
amcl_pid=$(pgrep -f \
  '^/opt/ros/humble/lib/nav2_amcl/amcl .*__node:=amcl( |$)')
printf 'NAVIGATION_READY: navigation_pid=%s amcl_pid=%s\n' "$navigation_pid" "$amcl_pid"
printf '%s\n' "$health_json"
echo 'NOTE: navigation chassis output remains disabled until explicitly enabled.'
navigation_started_by_script=false
ehi_started_by_script=false
