#!/usr/bin/env bash
set -eo pipefail

stop_old_processes() {
  local task_signal="$1"
  pkill "-${task_signal}" -f '[r]1pro_web_server' 2>/dev/null || true
  pkill "-${task_signal}" -f 'r1pro_web_control[.]head_depth_worker' 2>/dev/null || true
  pkill "-${task_signal}" -f 'r1pro_web_control[.]vision_worker' 2>/dev/null || true
  pkill "-${task_signal}" -f 'foundation_pose_ros2[.]foundation_pose_node' 2>/dev/null || true
  pkill "-${task_signal}" -f 'r1pro_aruco_box_pose.*aruco_box_pose[.]launch[.]py' 2>/dev/null || true
  pkill "-${task_signal}" -f 'r1pro_aruco_box_pose.*aruco_box_pose[.]py' 2>/dev/null || true
  pkill "-${task_signal}" -f 'r1pro_aruco_fine_approach.*aruco_fine_approach' 2>/dev/null || true
}

echo "Stopping old R1Pro Web and perception processes..."
stop_old_processes TERM
sleep 2
stop_old_processes KILL
sleep 1

if [[ -n "$(ss -ltnH 'sport = :8080')" ]]; then
  echo "Port 8080 is still occupied; refusing to start a conflicting Web server." >&2
  exit 1
fi

cd /home/motrix/galaxea/galaxea_overlay

source /opt/ros/humble/setup.bash
source /home/motrix/galaxea/install/setup.bash
source /home/motrix/galaxea/galaxea_overlay/install/setup.bash

export ROS_DOMAIN_ID=102
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_DDS_INTERFACE_IP=192.168.3.5
export R1PRO_FASTDDS_PROFILE=/home/motrix/galaxea/galaxea_overlay/install/r1pro_web_control/share/r1pro_web_control/config/fastdds_workstation_udp.xml
export FASTRTPS_DEFAULT_PROFILES_FILE="${R1PRO_FASTDDS_PROFILE}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE}"

exec ros2 run r1pro_web_control r1pro_web_server \
  --ros-args \
  -r __node:=r1pro_web_server \
  -p vla_policy_host:=10.30.20.73 \
  -p vla_policy_port:=18081 \
  -p head_depth_python:=/home/motrix/galaxea/FoundationPose/.venv/bin/python3 \
  -p head_depth_bundle_root:=/home/motrix/galaxea/galaxea_overlay/src/stereo_depth_offline_bundle \
  -p head_depth_output_timeout_s:=5.0 \
  "$@"
