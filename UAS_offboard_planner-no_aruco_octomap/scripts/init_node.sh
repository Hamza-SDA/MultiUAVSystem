#!/usr/bin/env bash

bold=$(tput bold)
normal=$(tput sgr0)

echo -n "Sourcing ${bold}UAS Offboard Planner${normal} ... "

PYTHON_PREFIX="python"
UBUNTU_RELEASE=$(lsb_release -rs)
if [[ "${UBUNTU_RELEASE}" == "14.04" ]]; then
  echo
  echo "Ubuntu 14.04 unsupported, see docker px4io/px4-dev-base"
  return 1
elif [[ "${UBUNTU_RELEASE}" == "16.04" ]]; then
  ROS_DISTRO="kinetic"
elif [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
  ROS_DISTRO="melodic"
elif [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
  ROS_DISTRO="noetic"
  PYTHON_PREFIX="python3"
fi

# Script directory
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# Workspace directory
WS_PATH=$(cd "$(dirname "$DIR")/../.." && pwd)

# Source Environments
## ROS Distro
source /opt/ros/${ROS_DISTRO}/setup.bash
## Catkin Workspace
source ${WS_PATH}/devel/setup.bash

echo "[DONE!]"
