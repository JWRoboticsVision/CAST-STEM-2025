#!/bin/bash
# author: Jikai Wang
# email: jikai.wang AT utdallas DOT edu

# Set the backend for Matplotlib
export MPLBACKEND=agg

# Set maximum number of jobs for the ninja build system
export MAX_JOBS=$(nproc)

# Get the current directory and project root
CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_ROOT=$(realpath "${CURR_DIR}/..")

# Conda Environment Name
CONDA_ENV_NAME="${PROJ_ROOT}/.env"

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

# Function to activate the Conda environment
activate_conda_env() {
  local env_name=$1

  if [ -z "${CONDA_PATH}" ]; then
    handle_error "CONDA_PATH is not set. Ensure Conda is installed and available in your PATH."
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
}

# Function to install Python packages and log progress
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

# Check if Conda is installed and get its base path
CONDA_PATH=$(command -v conda &>/dev/null && conda info --base 2>/dev/null)
if [ -z "${CONDA_PATH}" ]; then
  handle_error "Conda is not installed or not available in your PATH."
fi

# Activate the target Conda environment
activate_conda_env "${CONDA_ENV_NAME}"

# Set the Python executable path from the activated Conda environment
PYTHON_PATH="${CONDA_PREFIX}/bin/python"

# Validate the Python executable in the Conda environment
if [ ! -x "${PYTHON_PATH}" ]; then
  handle_error "Python executable not found in the Conda environment: ${PYTHON_PATH}"
fi
