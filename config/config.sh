#!/bin/bash
# author: Jikai Wang
# email: jikai.wang AT utdallas DOT edu

###################
# Get Paths
###################

CONFIG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_ROOT=$(realpath "${CONFIG_DIR}/..")
DOCKER_DIR=${PROJ_ROOT}/docker

# Python Path
PYTHON_PATH=$(which python3)

#####################
# Environment Variables
#####################

# Set the backend for Matplotlib
export MPLBACKEND=agg
# Set maximum number of jobs for the ninja build system
export MAX_JOBS=$(nproc)

#####################
# Conda Environment
#####################

# Conda Environment Name
CONDA_ENV_NAME="${PROJ_ROOT}/.env"

# Check if Conda is installed and get its base path
CONDA_PATH=$(command -v conda &>/dev/null && conda info --base 2>/dev/null)

###################
# Functions
###################

# Log file path for logging messages (default to /dev/null)
LOG_FILE="/dev/null"

# ---- Info Logging ----
log_message() {
  echo "[INFO] $(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

# ---- Error Logging ----
handle_error() {
  echo "[ERROR] $(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
  exit 1
}

# Function to activate the Conda environment
activate_conda_env() {
  local env_name=$1

  if [ -z "${CONDA_PATH}" ]; then
    handle_error "CONDA_PATH is not set. Ensure Conda is installed and available in your PATH."
  else
    echo "Conda found at: ${CONDA_PATH}"
  fi

  # Source conda.sh to enable `conda activate`
  source "${CONDA_PATH}/etc/profile.d/conda.sh" || handle_error "Failed to source conda.sh"

  # Activate the specified Conda environment
  conda activate "${env_name}" || handle_error "Failed to activate Conda environment: ${env_name}"

  # Check if CONDA_PREFIX is set after activation
  if [ -z "${CONDA_PREFIX}" ]; then
    handle_error "Conda environment not activated: ${env_name}"
  else
    log_message "Activated Conda environment: ${env_name}"
  fi

  # Set the Python executable path from the activated Conda environment
  if [ -x "${CONDA_PREFIX}/bin/python" ]; then
    PYTHON_PATH="${CONDA_PREFIX}/bin/python"
  else
    handle_error "Python executable not found in the Conda environment: ${PYTHON_PATH}"
  fi
}

# ---- install Python packages ----
install_python_package() {
  local package=$1
  local description=$2
  log_message "Installing ${description}..."
  if "${PYTHON_PATH}" -m pip install --no-cache-dir "$package"; then
    log_message "${description} installed successfully."
  else
    handle_error "Failed to install ${description}."
  fi
}

# ---- install requirements from a file ----
install_requirements() {
  local requirements_file=$1
  if [ -f "${requirements_file}" ]; then
    log_message "Installing Python packages from ${requirements_file}..."
    if "${PYTHON_PATH}" -m pip install --no-cache-dir -r "${requirements_file}"; then
      log_message "Requirements installed successfully."
    else
      handle_error "Failed to install requirements from ${requirements_file}."
    fi
  else
    handle_error "Requirements file not found: ${requirements_file}"
  fi
}
