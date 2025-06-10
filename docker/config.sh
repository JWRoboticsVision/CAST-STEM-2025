#!/bin/bash

DOCKER_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
PROJ_ROOT=$(dirname "$DOCKER_DIR")

#####################
# DOCKER BUILD ARGS
#####################

# Base image and CUDA version
BASE_DIST="ubuntu20.04"
CUDA_VERSION="11.8.0"

### GPU Compute Capability for RTX 2080,3090,4090 (ref to https://developer.nvidia.com/cuda-gpus)
TORCH_CUDA_ARCH_LIST="7.5 8.0 8.6 8.9+PTX"

# User configuration
USER_NAME="my_user"
USER_ID=$(id -u)
GROUP_ID=$(id -g)

# ROS1
ROS1_APT_PACKAGE="ros-noetic-desktop"

# ROS2
ROS2_APT_PACKAGE="ros-humble-desktop"

# ISAAC
ISAAC_LAB_VERSION="2.1.0"
ISAAC_SIM_VERSION="4.5.0"

###################
# Logging Functions
###################

# Log file path for logging messages (default to /dev/null)
LOG_FILE="/dev/null"

# Function to log messages with a timestamp
log_message() {
    echo "[INFO] $(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

# Function to handle errors and exit
handle_error() {
    echo "[ERROR] $(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
    exit 1
}
