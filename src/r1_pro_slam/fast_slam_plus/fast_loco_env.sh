#!/usr/bin/env bash
# Shared runtime environment for fast_loco.sh tmux windows.
set -Ee
set +u
ROOT_DIR=/home/nvidia/galaxea
STACK_ROOT="$ROOT_DIR/galaxea_overlay/src/r1_pro_slam/fast_slam_plus"
export ROS_DOMAIN_ID=102

source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
# Source only the installed Nav2 package, not the whole overlay: the latter
# contains a stale interface prefix that prints a missing local_setup warning.
NAV2_LOCAL_SETUP="$ROOT_DIR/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/local_setup.bash"
if [[ -f "$NAV2_LOCAL_SETUP" ]]; then
  source "$NAV2_LOCAL_SETUP"
fi
if [[ -f "$ROOT_DIR/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config/source_fastdds_simple.sh" ]]; then
  source "$ROOT_DIR/galaxea_overlay/install/r1_pro_navigation2/share/r1_pro_navigation2/config/source_fastdds_simple.sh"
fi
source "$STACK_ROOT/install/local_setup.bash"
set -u
