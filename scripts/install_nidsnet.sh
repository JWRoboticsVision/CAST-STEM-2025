#!/bin/bash

# Load shared configuration (variables and functions from config.sh)
source "$(dirname "$0")/config.sh"
NIDS_NET_DIR="${PROJ_ROOT}/third-party/NIDS-Net"

# Initialize the FoundationPose submodule
log_message "Checking NIDS-Net submodule..."
if [ -d "${NIDS_NET_DIR}" ]; then
  log_message "NIDS-Net submodule directory found: ${NIDS_NET_DIR}"
else
  handle_error "Please download the NIDS-Net code first!!!"
fi

# Install Python dependencies for NIDS-Net
log_message "Installing xformers 0.0.23..."
if "${PYTHON_PATH}" -m pip install --quiet --no-cache-dir "xformers==0.0.23"; then
  log_message "xformers installed successfully."
else
  handle_error "Failed to install xformers."
fi

log_message "Installing Python dependencies for NIDS-Net..."
if "${PYTHON_PATH}" -m pip install --quiet --no-cache-dir -r "${PROJ_ROOT}/requirements_nidsnet.txt"; then
  log_message "Python dependencies installed successfully."
else
  handle_error "Failed to install Python dependencies."
fi

# Install NIDS-Net package
log_message "Installing NIDS-Net package..."
if cd ${NIDS_NET_DIR} && "${PYTHON_PATH}" setup.py install; then
  log_message "NIDS-Net package installed successfully."
else
  handle_error "Failed to install NIDS-Net package."
fi

# Return to the project root directory
cd "$PROJ_ROOT" || handle_error "Failed to return to project root directory: $PROJ_ROOT"

log_message "All build steps completed successfully."
