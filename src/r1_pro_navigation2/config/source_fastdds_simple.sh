#!/usr/bin/env bash

# Source this file to restore the robot's default Fast DDS SIMPLE discovery.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "source ${BASH_SOURCE[0]}" >&2
  exit 2
fi

_r1pro_fastdds_config_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=102
export ROS_LOCALHOST_ONLY=0
# Compatibility metadata used by the existing R1 Pro startup environment.
# Fast DDS does not bind an interface from this variable; the XML transport
# profile is authoritative and intentionally keeps the robot's sensor LAN.
export ROS_DDS_INTERFACE_IP=192.168.3.4
export R1PRO_FASTDDS_DISCOVERY_MODE=simple

export R1PRO_FASTDDS_PROFILE="${_r1pro_fastdds_config_dir}/fastdds_robot_udp.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${R1PRO_FASTDDS_PROFILE}"
export FASTDDS_DEFAULT_PROFILES_FILE="${R1PRO_FASTDDS_PROFILE}"

unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset R1PRO_DISCOVERY_SERVER
unset _r1pro_fastdds_config_dir

echo "[r1pro-fastdds] mode=SIMPLE domain=${ROS_DOMAIN_ID} interface=${ROS_DDS_INTERFACE_IP} profile=${R1PRO_FASTDDS_PROFILE}"
