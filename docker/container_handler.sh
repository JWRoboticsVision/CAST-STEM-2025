#!/bin/bash
# author: Jikai Wang
# email: jikai.wang AT utdallas DOT edu

# Load external configuration
source $(dirname $0)/config.sh

export COMPOSE_PROJECT_NAME=${PROJECT_NAME}

# Check the .env file first
if [ ! -f ${DOCKER_DIR}/.env ]; then
  log_message "No .env file found. Please create one."
  exit 1
fi

# Input args
if [ "$#" -lt 1 ]; then
  log_message "Usage: $0 <case_name> [<profile_name>]"
  log_message "  case_name: build, run, enter, stop (default: build)"
  log_message "  profile_name: ros1-base, ros1-user (default: ros1-base)"
  exit 1
fi

CASE_NAME=${1:-build}
PROFILE_NAME=${2:-ros1-base}

# Validate inputs
if [[ ! "$CASE_NAME" =~ ^(build|run|enter|stop)$ ]]; then
  log_message "Invalid case name: $CASE_NAME. Valid: build, run, enter, stop."
  exit 1
fi

if [[ ! "$PROFILE_NAME" =~ ^(ros1-base|ros1-user)$ ]]; then
  log_message "Invalid profile name: $PROFILE_NAME. Valid: ros1-base, ros1-user."
  exit 1
fi

# Determine container name and image tag
if [[ "$PROFILE_NAME" == "ros1-base" ]]; then
  CONTAINER_NAME="ubuntu20.04-ros-noetic-base"
  IMAGE_TAG="irvlutd/ubuntu20.04-ros-noetic-base:latest"
elif [[ "$PROFILE_NAME" == "ros1-user" ]]; then
  # CONTAINER_NAME="ubuntu20.04-ros-noetic-${HOST_UID}"
  # IMAGE_TAG="irvlutd/ubuntu20.04-ros-noetic:${HOST_UID}"
  # CONTAINER_NAME="ubuntu20.04-ros-noetic-user"
  # IMAGE_TAG="irvlutd/ubuntu20.04-ros-noetic-user:latest"
  CONTAINER_NAME="ubuntu20.04-ros-noetic"
  IMAGE_TAG="irvlutd/ubuntu20.04-ros-noetic:latest"
fi

###################
# Functions
###################

# Dynamic container status
get_container_status() {
  if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "running"
  else
    echo "stopped"
  fi
}

# ---- Build ----
build_container() {
  local status
  status=$(get_container_status)
  if [ "$status" == "running" ]; then
    log_message "Container $CONTAINER_NAME is running. Please stop it before building."
    exit 1
  fi

  log_message "Building image $IMAGE_TAG..."
  docker compose --env-file ${DOCKER_DIR}/.env --file ${DOCKER_DIR}/docker-compose.yaml --profile $PROFILE_NAME build
}

# ---- Run ----
run_container() {
  local status
  status=$(get_container_status)
  if [ "$status" == "running" ]; then
    log_message "Container $CONTAINER_NAME is already running."
    exit 1
  fi

  log_message "Running container $CONTAINER_NAME..."
  docker compose \
    --env-file ${DOCKER_DIR}/.env \
    --file ${DOCKER_DIR}/docker-compose.yaml \
    --profile ${PROFILE_NAME} up -d
}

# ---- Enter ----
enter_container() {
  local status
  status=$(get_container_status)
  if [ "$status" != "running" ]; then
    log_message "Container $CONTAINER_NAME is not running. Please start it first."
    exit 1
  fi

  log_message "Entering container $CONTAINER_NAME..."
  docker exec --user my_user -it $CONTAINER_NAME zsh --login
}

# ---- Stop ----
stop_container() {
  local status
  status=$(get_container_status)
  if [ "$status" == "running" ]; then
    log_message "Stopping container $CONTAINER_NAME..."
    docker compose \
      --env-file ${DOCKER_DIR}/.env \
      --file ${DOCKER_DIR}/docker-compose.yaml \
      --profile ${PROFILE_NAME} down
  else
    log_message "Container $CONTAINER_NAME is not running."
  fi
}

# ---- Dispatcher ----
case "$CASE_NAME" in
build) build_container ;;
run) run_container ;;
enter) enter_container ;;
stop) stop_container ;;
esac
