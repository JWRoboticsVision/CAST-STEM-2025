#!/bin/bash
set -e

DOCKER_DEFAULT_USER="my_user"
DEFAULT_UID=1000
DEFAULT_GID=1000

HOST_UID="${HOST_UID:-$DEFAULT_UID}"
HOST_GID="${HOST_GID:-$DEFAULT_GID}"

echo "Setting up container for UID: $HOST_UID, GID: $HOST_GID"

# --- Group setup ---
EXISTING_GROUP_BY_GID=$(getent group "$HOST_GID" | cut -d: -f1)
EXISTING_GROUP_BY_NAME=$(getent group "$DOCKER_DEFAULT_USER")

if [ -z "$EXISTING_GROUP_BY_GID" ]; then
  if [ -z "$EXISTING_GROUP_BY_NAME" ]; then
    echo "Creating group '$DOCKER_DEFAULT_USER' with GID $HOST_GID..."
    groupadd -g "$HOST_GID" "$DOCKER_DEFAULT_USER"
  else
    echo "Group '$DOCKER_DEFAULT_USER' already exists, but not with GID $HOST_GID. Renaming it..."
    groupmod -g "$HOST_GID" "$DOCKER_DEFAULT_USER"
  fi
else
  echo "Group with GID $HOST_GID already exists (name: $EXISTING_GROUP_BY_GID). Using it."
fi

# --- User setup ---
EXISTING_USER_BY_UID=$(getent passwd "$HOST_UID" | cut -d: -f1)

if [ -z "$EXISTING_USER_BY_UID" ]; then
  echo "No existing user with UID $HOST_UID. Modifying default user '$DOCKER_DEFAULT_USER'..."
  usermod -u "$HOST_UID" -g "$HOST_GID" -s /bin/zsh "$DOCKER_DEFAULT_USER"
else
  echo "User '$EXISTING_USER_BY_UID' with UID $HOST_UID already exists."
  usermod -g "$HOST_GID" -s /bin/zsh "$EXISTING_USER_BY_UID"
  if [ "$EXISTING_USER_BY_UID" != "$DOCKER_DEFAULT_USER" ]; then
    echo "Renaming user $EXISTING_USER_BY_UID to $DOCKER_DEFAULT_USER..."
    usermod -l "$DOCKER_DEFAULT_USER" "$EXISTING_USER_BY_UID"
  fi
fi

# --- Directory ownership ---
echo "Fixing ownership of /home/$DOCKER_DEFAULT_USER and /opt/conda..."
chown -R "$HOST_UID:$HOST_GID" "/home/$DOCKER_DEFAULT_USER"
chown -R "$HOST_UID:$HOST_GID" /opt/conda

# --- Environment ---
export HOME="/home/$DOCKER_DEFAULT_USER"

# --- Prepare privilege drop ---
if [ "$(id -u)" -eq 0 ]; then
  echo "Switching to user '$DOCKER_DEFAULT_USER' using gosu..."
  GOSU_CMD="gosu $DOCKER_DEFAULT_USER"
else
  echo "Already running as non-root user $(whoami)"
  GOSU_CMD=""
fi

# --- Oh My Zsh permission fix ---
if [ -d "$HOME/.oh-my-zsh" ]; then
  echo "Fixing permissions for Oh My Zsh..."
  find "$HOME/.oh-my-zsh" -type d -exec chmod go-w {} +
  chown -R "$HOST_UID:$HOST_GID" "$HOME/.oh-my-zsh"
fi

# --- Final command ---
if [ "$#" -eq 0 ]; then
  FINAL_COMMAND="zsh"
else
  FINAL_COMMAND="$@"
fi

# --- Optional NVIDIA entrypoint ---
NVIDIA_ENTRYPOINT="/opt/nvidia/nvidia_entrypoint.sh"

# --- Final command logic ---
if [ -x "$NVIDIA_ENTRYPOINT" ]; then
  echo "Using NVIDIA entrypoint: $NVIDIA_ENTRYPOINT"

  if [ "$#" -eq 0 ] || [ "$FINAL_COMMAND" = "zsh" ]; then
    exec $GOSU_CMD "$NVIDIA_ENTRYPOINT" /bin/bash -c "exec zsh --login"
  else
    exec $GOSU_CMD "$NVIDIA_ENTRYPOINT" /bin/bash -c "exec zsh -ic '${FINAL_COMMAND}'"
  fi
else
  if [ "$#" -eq 0 ] || [ "$FINAL_COMMAND" = "zsh" ]; then
    exec $GOSU_CMD zsh --login
  else
    exec $GOSU_CMD zsh -ic "${FINAL_COMMAND}"
  fi
fi

# Change to my_user
if [ "$(id -u)" -eq 0 ]; then
  echo "Switching to user '$DOCKER_DEFAULT_USER' using gosu..."
  exec gosu "$DOCKER_DEFAULT_USER" "$FINAL_COMMAND"
else
  echo "Already running as non-root user $(whoami)"
  exec "$FINAL_COMMAND"
fi
