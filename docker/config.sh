#!/bin/bash
# author: Jikai Wang
# email: jikai.wang AT utdallas DOT edu

###################
# Source Config
###################

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../config/config.sh"

#####################
# DOCKER ARGS
#####################

# Base image and CUDA version
CUDA_VERSION="11.8.0"

### GPU Compute Capability for RTX 2080,3090,4090 (ref to https://developer.nvidia.com/cuda-gpus)
TORCH_CUDA_ARCH_LIST="7.5 8.0 8.6 8.9+PTX"

# User configuration
USER_NAME="my_user"
HOST_UID=$(id -u)
HOST_GID=$(id -g)

# ROS
ROS1_DISTRO="noetic"
ROS2_DISTRO="humble"

# Compose project name
PROJECT_NAME="summer_camp-${HOST_UID}"

# Miniconda version
# reference: https://repo.anaconda.com/miniconda/
MINICONDA_VERSION="latest"

# Timezone
ZONE_PATH=$(readlink -f /etc/localtime)
if [[ "$ZONE_PATH" == /usr/share/zoneinfo/* ]]; then
  TZ=$(realpath --relative-to=/usr/share/zoneinfo "$ZONE_PATH")
else
  TZ="Etc/UTC"
fi
