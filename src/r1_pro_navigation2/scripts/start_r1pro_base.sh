#!/usr/bin/env bash

# Complete, fail-stop R1PRO base restart.  This intentionally does not enable
# navigation or restart EHI; start_r1pro_navigation.sh does that after AMCL is
# healthy.
set -Ee -o pipefail

trap 'rc=$?; printf "ERROR: base startup stopped at line %s (rc=%s)\n" "$LINENO" "$rc" >&2; exit "$rc"' ERR

DDS_CONFIG=/home/nvidia/galaxea/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config
STARTUP_DIR=/home/nvidia/galaxea/install/startup_config/share/startup_config/script
SESSION_CONFIG_ROOT=/home/nvidia/galaxea/install/startup_config/share/startup_config/sessions.d/ATCStandard
START_TYPE_FILE=/tmp/start_type.log
BODY_MOBIMAN_CONFIG="$SESSION_CONFIG_ROOT/R1PROBody.d/mobiman.yaml"
TORSO_KEY_ENV=/opt/galaxea/galaxeaKey/keys.env
ARUCO_RELAY_SERVICE=r1pro-aruco-camera-relay.service

if [ -n "${TMUX:-}" ]; then
  echo 'ERROR: run from a normal SSH/local terminal, not from inside tmux' >&2
  exit 1
fi

test -r "$DDS_CONFIG/source_fastdds_simple.sh"
test -r "$DDS_CONFIG/fastdds_robot_udp.xml"
test -x "$STARTUP_DIR/robot_startup.sh"
test -r "$TORSO_KEY_ENV"

# R1PROBody intentionally starts the joint tracker with torso control enabled.
# Fail before stopping the running robot if a vendor reinstall has reverted the
# preset to the helper's protected default (disable_torso=true).
if ! grep -Fq -- './start_mobiman_r1pro_joint_pid.sh slow enable_torso' \
  "$BODY_MOBIMAN_CONFIG"; then
  echo 'ERROR: R1PROBody torso control is not enabled in mobiman.yaml' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash
source /home/nvidia/galaxea/galaxea_overlay/install/setup.bash
source "$DDS_CONFIG/source_fastdds_simple.sh"

# Stop the user-level camera relay before auditing and cleaning Fast DDS shared
# memory. It is restarted after the base health checks pass below.
systemctl --user stop "$ARUCO_RELAY_SERVICE" 2>/dev/null || true

for required_command in awk fastdds grep ip lsof pgrep ss sudo systemctl timeout tmux; do
  command -v "$required_command" >/dev/null || {
    echo "ERROR: required command not found: $required_command" >&2
    exit 1
  }
done

sudo -v

mapfile -t existing_sessions < <(
  tmux list-sessions -F '#{session_name}' 2>/dev/null || true
)

# robot_startup.sh records the active startup preset in START_TYPE_FILE.  Build
# the allowlist from that preset and the R1PROBody target instead of assuming
# that every robot preset creates the same tmux sessions.  This keeps the
# fail-stop protection for unrelated tmux sessions while allowing a supported
# VR -> Body transition (VR also owns system, vr_teleop and websocket_bridge).
declare -A allowed_robot_sessions=([navigation]=1)

allow_preset_sessions() {
  local preset_dir=$1
  local declared_session

  test -d "$preset_dir"
  while IFS= read -r declared_session; do
    [ -n "$declared_session" ] && allowed_robot_sessions["$declared_session"]=1
  done < <(awk '$1 == "session_name:" {print $2}' "$preset_dir"/*.yaml)
}

target_preset_dir="$SESSION_CONFIG_ROOT/R1PROBody.d"
allow_preset_sessions "$target_preset_dir"

current_start_type=''
if [ -r "$START_TYPE_FILE" ]; then
  IFS= read -r current_start_type < "$START_TYPE_FILE" || true
fi
if [ -n "$current_start_type" ]; then
  if [[ ! "$current_start_type" =~ ^[A-Za-z0-9_]+$ ]]; then
    echo "ERROR: invalid current startup type '$current_start_type'" >&2
    exit 1
  fi
  current_preset_dir="$SESSION_CONFIG_ROOT/$current_start_type.d"
  if [ ! -d "$current_preset_dir" ]; then
    echo "ERROR: recorded startup preset does not exist: $current_preset_dir" >&2
    exit 1
  fi
  if [ "$current_preset_dir" != "$target_preset_dir" ]; then
    allow_preset_sessions "$current_preset_dir"
  fi
fi

for session_name in "${existing_sessions[@]}"; do
  if [ -z "${allowed_robot_sessions[$session_name]+x}" ]; then
    echo "ERROR: unexpected tmux session '$session_name'; restart refused" >&2
    exit 1
  fi
done

printf 'Stopping robot tmux sessions: %s\n' "${existing_sessions[*]:-(none)}"

timeout 5 ros2 service call \
  /cmd_vel_to_twist_stamped/set_nav_output_enabled \
  std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1 || true

sudo systemctl stop ehi-gateway.service
if systemctl is-active --quiet ehi-gateway.service; then
  echo 'ERROR: ehi-gateway.service did not stop' >&2
  exit 1
fi

sudo systemctl disable --now fastdds-discovery.service
test "$(systemctl show fastdds-discovery.service -p UnitFileState --value)" = disabled
test "$(systemctl show fastdds-discovery.service -p ActiveState --value)" = inactive
test "$(systemctl show fastdds-discovery.service -p SubState --value)" = dead

cd "$STARTUP_DIR"
if [ "${#existing_sessions[@]}" -gt 0 ]; then
  ./robot_startup.sh kill
else
  ros2 daemon stop >/dev/null 2>&1 || true
fi
ros2 daemon stop >/dev/null 2>&1 || true

# Clean only after proving no process has a Fast DDS SHM file open.
fastdds_open=''
for _attempt in $(seq 1 15); do
  if lsof_output=$(sudo lsof -nP +D /dev/shm 2>&1); then
    lsof_rc=0
  else
    lsof_rc=$?
  fi
  if [ "$lsof_rc" -ne 0 ]; then
    if [ "$lsof_rc" -eq 1 ] && [ -z "$lsof_output" ]; then
      fastdds_open=''
      break
    fi
    echo "ERROR: cannot audit /dev/shm with lsof (rc=$lsof_rc)" >&2
    printf '%s\n' "$lsof_output" >&2
    exit 1
  fi
  fastdds_open=$(printf '%s\n' "$lsof_output" | \
    awk 'NR > 1 && $0 ~ /\/dev\/shm\/(fastrtps_|sem\.fastrtps_)/ {print}')
  [ -z "$fastdds_open" ] && break
  sleep 1
done
if [ -n "$fastdds_open" ]; then
  echo 'ERROR: Fast DDS SHM is still held; cleanup refused:' >&2
  printf '%s\n' "$fastdds_open" >&2
  exit 1
fi

# A process that survived robot_startup.sh kill must never be joined by a
# second base stack, even if it happens not to hold a matching SHM file.
for stale_pattern in \
  '^/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher .*__node:=robot_state_publisher( |$)' \
  '^/home/nvidia/galaxea/install/mobiman/lib/mobiman/chassis_control_node( |$)' \
  '^/home/nvidia/galaxea/install/HDAS/lib/HDAS/HDAS( |$)' \
  '^/opt/ros/humble/bin/fast-discovery-server .* -p 11811( |$)' \
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch r1_pro_navigation2 body_attach\.launch\.py( |$)' \
  '^/opt/ros/humble/lib/nav2_amcl/amcl .*__node:=amcl( |$)' \
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch mobiman r1_pro_jointTrackerdemo_launch\.py( |$)' \
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch mobiman torso_control_r1\.launch\.py( |$)' \
  '^/home/nvidia/galaxea/install/mobiman/lib/mobiman/mobiman_torso_control_example( |$)' \
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch teleoperation_ros2 vr_teleoperation(_whole_body)?\.launch\.py( |$)' \
  '^/home/nvidia/galaxea/install/teleoperation_ros2/lib/teleoperation_ros2/vr_vr_controller_node( |$)'; do
  mapfile -t stale_pids < <(pgrep -f "$stale_pattern" || true)
  if [ "${#stale_pids[@]}" -gt 0 ]; then
    echo "ERROR: old robot process survived shutdown: ${stale_pids[*]}" >&2
    ps -o pid=,ppid=,stat=,args= -p "${stale_pids[@]}" >&2 || true
    exit 1
  fi
done
fastdds shm clean

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.875 \
  dbitrate 5000000 fd on dsample-point 0.875
sudo ip link set can0 up

./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/

wait_for_tmux() {
  local session_name=$1
  for _attempt in $(seq 1 40); do
    tmux has-session -t "$session_name" 2>/dev/null && return 0
    sleep 1
  done
  echo "ERROR: tmux session '$session_name' did not start" >&2
  return 1
}

for expected_session in hdas mobiman ros_discovery tools; do
  wait_for_tmux "$expected_session"
done

wait_for_unique_process() {
  local pattern=$1
  local label=$2
  local count
  for _attempt in $(seq 1 40); do
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
  '^/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher .*__node:=robot_state_publisher( |$)' \
  robot_state_publisher
wait_for_unique_process \
  '^/home/nvidia/galaxea/install/mobiman/lib/mobiman/chassis_control_node( |$)' \
  chassis_control_node
wait_for_unique_process \
  '^/home/nvidia/galaxea/install/HDAS/lib/HDAS/HDAS( |$)' \
  HDAS
wait_for_unique_process \
  '^/opt/ros/humble/bin/fast-discovery-server .* -p 11811( |$)' \
  fast-discovery-server
wait_for_unique_process \
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch mobiman r1_pro_jointTrackerdemo_launch\.py .*disable_torso:=false( |$)' \
  torso-enabled-joint-tracker

ss_output=$(ss -Hlunp)
if ! grep -Eq ':11811([[:space:]]|$).*fast-discovery-' <<<"$ss_output"; then
  echo 'ERROR: Discovery Server is not listening on UDP 11811' >&2
  exit 1
fi

tmux list-sessions
systemctl --user start "$ARUCO_RELAY_SERVICE"
if ! systemctl --user is-active --quiet "$ARUCO_RELAY_SERVICE"; then
  echo "ERROR: $ARUCO_RELAY_SERVICE did not start" >&2
  exit 1
fi
echo "ARUCO_RELAY_READY: $ARUCO_RELAY_SERVICE is running"
echo 'BASE_READY: base configuration is running; now run: bash ~/start_r1pro_navigation.sh'
echo 'NOTE: ehi-gateway remains stopped until navigation becomes healthy.'
