#!/usr/bin/env bash
set -eo pipefail

cd /home/motrix/galaxea/galaxea_overlay

source /opt/ros/humble/setup.bash
source /home/motrix/galaxea/install/setup.bash
source /home/motrix/galaxea/galaxea_overlay/install/setup.bash

export ROS_DOMAIN_ID=102
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_DDS_INTERFACE_IP=192.168.3.5
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/motrix/galaxea/install/startup_config/share/startup_config/script/prestart/fastrtps_profiles.xml
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "ROS_DDS_INTERFACE_IP=${ROS_DDS_INTERFACE_IP}"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo "FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE}"

find_existing_web_pids() {
  ps -eo pid=,args= | awk -v self="$$" '
    $1 == self { next }
    $0 ~ /(^|\/)awk / { next }
    index($0, "ros2 launch r1pro_web_control") ||
    index($0, "install/r1pro_web_control/lib/r1pro_web_control/r1pro_web_server") ||
    index($0, "__node:=r1pro_web_server") ||
    index($0, "__node:=r1pro_web_left_wrist_camera_tf_bridge") ||
    index($0, "__node:=r1pro_web_right_wrist_camera_tf_bridge") {
      print $1
    }
  '
}

pids="$(find_existing_web_pids | tr '\n' ' ')"
if [ -n "${pids// }" ]; then
  echo "Stopping existing r1pro web processes: ${pids}"
  kill -TERM ${pids} 2>/dev/null || true
  sleep 2
  pids="$(find_existing_web_pids | tr '\n' ' ')"
  if [ -n "${pids// }" ]; then
    kill -KILL ${pids} 2>/dev/null || true
    sleep 1
  fi
fi

exec ros2 run r1pro_web_control r1pro_web_server \
  --ros-args -r __node:=r1pro_web_server "$@"
