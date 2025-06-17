#!/bin/bash

source $(dirname $0)/config.sh

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
  ros1-base:
    profiles:
      - ros1-base
    container_name: ubuntu20.04-ros-noetic-base
    image: irvlutd/ubuntu20.04-ros-noetic-base:latest
    build:
      context: .
      dockerfile: Dockerfile.ros1-base
      args:
        - BASE_DIST_ARG=ubuntu20.04
        - CUDA_VERSION_ARG=${CUDA_VERSION}
        - TORCH_CUDA_ARCH_LIST_ARG=${TORCH_CUDA_ARCH_LIST}
        - ROS1_APT_PACKAGES_ARG=${ROS1_APT_PACKAGE}
        - CONDA_VERSION_ARG=${MINICONDA_VERSION}
    environment: *default-base-environment
    volumes: *default-base-volumes
    deploy: *default-base-deploy
    network_mode: host
    cap_add:
      - SYS_PTRACE
    security_opt:
      - seccomp=unconfined
    privileged: true
    stdin_open: true
    tty: true

  # This service adds user-specific configuration
  # on top of the ros1 base image
  ros1-user:
    profiles:
      - ros1-user
    container_name: ubuntu20.04-ros-noetic
    image: irvlutd/ubuntu20.04-ros-noetic:latest
    build:
      context: .
      # dockerfile: Dockerfile.ros1-user
      dockerfile: Dockerfile.ros1
      args:
        - BASE_DIST_ARG=ubuntu20.04
        - CUDA_VERSION_ARG=${CUDA_VERSION}
        - TORCH_CUDA_ARCH_LIST_ARG=${TORCH_CUDA_ARCH_LIST}
        - ROS1_APT_PACKAGES_ARG=${ROS1_APT_PACKAGE}
        - CONDA_VERSION_ARG=${MINICONDA_VERSION}
        - USER_NAME_ARG=my_user
        - USER_UID_ARG=1000
        - USER_GID_ARG=1000
    environment:
      <<: *default-base-environment
      HOST_UID: ${HOST_UID}
      HOST_GID: ${HOST_GID}
    volumes: *default-user-volumes
    deploy: *default-base-deploy
    network_mode: host
    privileged: true
    stdin_open: true
    tty: true
EOF

log_message "Done!!!"
