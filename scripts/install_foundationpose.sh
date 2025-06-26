#!/bin/bash

# Load shared configuration (variables and functions from config.sh)
source "$(dirname "$0")/config.sh"
FD_POSE_DIR="${PROJ_ROOT}/third-party/FoundationPose"

# Initialize the FoundationPose submodule
log_message "Checking FoundationPose submodule..."
if [ -d "${FD_POSE_DIR}" ]; then
  log_message "FoundationPose submodule directory found: ${FD_POSE_DIR}"
else
  handle_error "Please download the FoundationPose code first!!!"
fi

# Install Python dependencies from requirements_fdpose.txt
log_message "Installing Python dependencies from requirements_fdpose.txt..."
if "${PYTHON_PATH}" -m pip install --quiet --no-cache-dir -r "${PROJ_ROOT}/requirements_fdpose.txt"; then
  log_message "Python dependencies installed successfully."
else
  handle_error "Failed to install Python dependencies."
fi

# Install NVDiffRast
log_message "Installing NVDiffRast..."
if "${PYTHON_PATH}" -m pip install --quiet --no-cache-dir "git+https://github.com/NVlabs/nvdiffrast.git@v0.3.3"; then
  log_message "NVDiffRast installed successfully."
else
  handle_error "Failed to install NVDiffRast."
fi

# Install PyTorch3D
log_message "Installing PyTorch3D..."
if "${PYTHON_PATH}" -m pip install --quiet --no-cache-dir "git+https://github.com/facebookresearch/pytorch3d.git@V0.7.8"; then
  log_message "PyTorch3D installed successfully."
else
  handle_error "Failed to install PyTorch3D."
fi

# Set paths for mycpp build
log_message "Preparing to build mycpp..."
MYCPP_DIR="${FD_POSE_DIR}/mycpp"
BUILD_DIR="${MYCPP_DIR}/build"
CMAKE_PREFIX_PATH="${CONDA_PREFIX}/lib/python3.1/site-packages/pybind11/share/cmake/pybind11"

# Check if CMAKE_PREFIX_PATH exists
if [ ! -d "$CMAKE_PREFIX_PATH" ]; then
  handle_error "CMAKE_PREFIX_PATH does not exist at $CMAKE_PREFIX_PATH"
fi

# Remove existing build directory if it exists
if [ -d "$BUILD_DIR" ]; then
  log_message "Removing existing build directory: $BUILD_DIR"
  rm -rf "$BUILD_DIR"
fi

# Create build directory and navigate to it
log_message "Creating build directory: $BUILD_DIR"
mkdir -p "$BUILD_DIR" && cd "$BUILD_DIR" || handle_error "Failed to create or navigate to build directory: $BUILD_DIR"

# Run CMake
log_message "Running CMake configuration..."
if cmake -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH" ..; then
  log_message "CMake configuration successful."
else
  handle_error "CMake configuration failed."
fi

# Run Make
log_message "Running Make build with $MAX_JOBS jobs..."
if make -j"$MAX_JOBS"; then
  log_message "mycpp built successfully."
else
  handle_error "Make build failed."
fi

# Return to the project root directory
cd "$PROJ_ROOT" || handle_error "Failed to return to project root directory: $PROJ_ROOT"

log_message "All build steps completed successfully."
