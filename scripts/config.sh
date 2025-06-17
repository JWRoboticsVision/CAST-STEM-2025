#!/bin/bash
# author: Jikai Wang
# email: jikai.wang AT utdallas DOT edu

###################
# Source Config
###################

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../config/config.sh"

###################
# Activate Conda Environment
###################

activate_conda_env "${CONDA_ENV_NAME}"
