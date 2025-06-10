#!/bin/bash

source $(dirname $0)/config.sh

# Create the conda env directory if it doesn't exist
if [ ! -d "${PROJ_ROOT}/.env" ]; then
  mkdir -p ${PROJ_ROOT}/.env
fi

# Generate a .env file for Docker build arguments
log_message "Generating .env file for Docker build arguments..."
touch ${DOCKER_DIR}/.env

# Generate the docker-compose.yaml file
log_message "Generating docker-compose.yaml file..."
cat <<EOF >${DOCKER_DIR}/docker-compose.yaml
# Reusable extensions
x-default-base-volumes: &default-base-volumes
  - type: bind
    source: /tmp/.X11-unix
    target: /tmp/.X11-unix
  - type: bind
    source: ${PROJ_ROOT}/docker/ros/.ros
    target: /home/${USER_NAME}/.ros

x-default-user-volumes: &default-user-volumes
  - <<: *default-base-volumes
  # Project root folder
  - type: bind
    source: ${PROJ_ROOT}
    target: /home/${USER_NAME}/code
  # ROS1 workspace
  - type: bind
    source: ${PROJ_ROOT}/docker/ros/catkin_ws
    target: /home/${USER_NAME}/catkin_ws
  # NFS folder if available
  - type: bind
    source: /srv
    target: /srv

x-default-base-environment: &default-base-environment
  DISPLAY: ${DISPLAY}
  TERM: ${TERM}
  QT_X11_NO_MITSHM: 1
  TZ: ${TZ}

x-default-base-deploy: &default-base-deploy
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: all
          capabilities: [ gpu ]

services:
  ros1-base:
    profiles:
      - ros1-base
    container_name: ubuntu20.04-ros-noetic-base
    image: irvlutd/ubuntu20.04-ros-noetic-base:latest
    build:
      context: .
      dockerfile: Dockerfile.ros1-base
      args:
        - TORCH_CUDA_ARCH_LIST_ARG=${TORCH_CUDA_ARCH_LIST}
        - ROS1_APT_PACKAGES_ARG=${ROS1_APT_PACKAGE}
    environment: *default-base-environment
    volumes: *default-base-volumes
    deploy: *default-base-deploy
    network_mode: host
    # devices:
    #   - "/dev:/dev"
    cap_add:
      - SYS_PTRACE
    security_opt:
      - seccomp=unconfined
    privileged: true
    stdin_open: true
    tty: true

  # This service adds user-specific configuration
  # on top of the ros1 base image
  ros1-${USER_ID}:
    profiles:
      - ros1-user
    container_name: ubuntu20.04-ros-noetic-${USER_ID}
    image: irvlutd/ubuntu20.04-ros-noetic:${USER_ID}
    build:
      context: .
      dockerfile: Dockerfile.ros1-user
      args:
        - USER_NAME_ARG=${USER_NAME}
        - USER_UID_ARG=${USER_ID}
        - USER_GID_ARG=${GROUP_ID}
    environment: *default-base-environment
    volumes: *default-user-volumes
    deploy: *default-base-deploy
    network_mode: host
    privileged: true
    stdin_open: true
    tty: true
EOF

log_message "Done!!!"
