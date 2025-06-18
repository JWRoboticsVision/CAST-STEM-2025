#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/config.sh"

# Generate a .env file for Docker build arguments
log_message "Generating .env file for Docker build arguments..."
touch ${DOCKER_DIR}/.env

# Generate the docker-compose.yaml file
log_message "Generating docker-compose.yaml file..."
cat <<EOF >${DOCKER_DIR}/docker-compose.yaml
x-default-base-volumes: &default-base-volumes
  - type: bind
    source: /tmp/.X11-unix
    target: /tmp/.X11-unix
  # NFS folder if available
  - type: bind
    source: /srv
    target: /srv

x-default-user-volumes: &default-user-volumes
  - <<: *default-base-volumes
  # Project root folder
  - type: bind
    source: ${PROJ_ROOT}
    target: /home/${USER_NAME}/code
  # ROS Mapping
  - type: bind
    source: ${PROJ_ROOT}/docker/ros/catkin_ws
    target: /home/${USER_NAME}/catkin_ws
  - type: bind
    source: ${PROJ_ROOT}/docker/ros/.ros
    target: /home/${USER_NAME}/.ros
  - type: bind
    source: ${PROJ_ROOT}/docker/ros/.gazebo
    target: /home/${USER_NAME}/.gazebo

x-default-base-environment: &default-base-environment
  DISPLAY: ${DISPLAY}
  TERM: ${TERM}
  QT_X11_NO_MITSHM: 1
  # Timezone
  TZ: ${TZ}

x-default-base-deploy: &default-base-deploy
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: all
          capabilities: [ gpu ]

services:
  ros1-user:
    profiles:
      - ros1-user
    container_name: ubuntu20.04-cuda-${CUDA_VERSION}-ros-${ROS1_DISTRO}-${HOST_UID}
    image: irvlutd/ubuntu20.04-cuda-${CUDA_VERSION}-ros-${ROS1_DISTRO}:latest
    build:
      context: .
      dockerfile: Dockerfile.ros1
      args:
        - BASE_DIST_ARG=ubuntu20.04
        - CUDA_VERSION_ARG=${CUDA_VERSION}
        - TORCH_CUDA_ARCH_LIST_ARG=${TORCH_CUDA_ARCH_LIST}
        - ROS1_APT_PACKAGES_ARG=${ROS1_DISTRO}
        - CONDA_VERSION_ARG=${MINICONDA_VERSION}
        - USER_NAME_ARG=my_user
        - USER_UID_ARG=1000
        - USER_GID_ARG=1000
    environment:
      <<: *default-base-environment
      HOST_UID: ${HOST_UID}
      HOST_GID: ${HOST_GID}
    volumes: *default-user-volumes
    working_dir: /home/${USER_NAME}/code
    deploy: *default-base-deploy
    network_mode: host
    privileged: true
    stdin_open: true
    tty: true
EOF

log_message "Done!!!"
