#!/usr/bin/env bash

# Source this file only for ROS graph introspection (ros2 CLI, rqt_graph, etc.).
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
export R1PRO_FASTDDS_DISCOVERY_MODE=client
# Robot diagnostics always use the server on this host.  Assign explicitly so
# a stale value inherited from an external-web shell cannot redirect discovery.
export R1PRO_DISCOVERY_SERVER=127.0.0.1:11811

# SUPER_CLIENT receives the complete discovery graph and should not be used by
# normal robot nodes, because it deliberately disables the metadata reduction.
export R1PRO_FASTDDS_PROFILE="${_r1pro_fastdds_config_dir}/fastdds_robot_udp.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${R1PRO_FASTDDS_PROFILE}"
export FASTDDS_DEFAULT_PROFILES_FILE="${R1PRO_FASTDDS_PROFILE}"
export ROS_DISCOVERY_SERVER="${R1PRO_DISCOVERY_SERVER}"
export ROS_SUPER_CLIENT=TRUE
unset _r1pro_fastdds_config_dir

echo "[r1pro-fastdds] mode=SUPER_CLIENT domain=${ROS_DOMAIN_ID} interface=${ROS_DDS_INTERFACE_IP} server=${ROS_DISCOVERY_SERVER} profile=${R1PRO_FASTDDS_PROFILE}"
