# Container Installation

## Table of Contents

- [Container Installation](#container-installation)
  - [Table of Contents](#table-of-contents)
  - [Container Setup](#container-setup)
      - [1. Install Docker](#1-install-docker)
      - [2. Install the NVIDIA Container Toolkit](#2-install-the-nvidia-container-toolkit)
  - [Building the Container](#building-the-container)
      - [1. Generate the `.env` file](#1-generate-the-env-file)
      - [2. Build the ros1-base container](#2-build-the-ros1-base-container)
      - [3. Build the ros1-user container](#3-build-the-ros1-user-container)

## Container Setup

#### 1. Install Docker

```bash
# Docker installation using the convenience script
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Post-install steps for Docker
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker

# Verify Docker
docker run hello-world
```

#### 2. Install the NVIDIA Container Toolkit

```bash
# Configure the repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
    && \
    sudo apt-get update

# Install the NVIDIA Container Toolkit packages
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Configure the container runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Verify NVIDIA Container Toolkit
docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

## Building the Container

The provided Dockerfile builds containers with the following specifications:

- **ros1-base Container:**
  - Based on `nvidia/cuda:11.8.0-devel-ubuntu20.04`, which includes:
    - CUDA 11.8 development environment
    - Ubuntu 22.04 as the base operating system
  - Graphics Support:
    - Preconfigured with `OpenGL` and the `Vulkan SDK` for hardware-accelerated rendering and simulation.
  - ROS Noetic Support:
    - Installs ROS Noetic, which is compatible with Ubuntu 20.04.
  - Python Environment:
    - Installs Miniconda3 for Python package management and environment isolation.
    - Configures the environment with the conda-forge channel and strict channel priority.
- **ros1-user Container:**
  - Builds on the ros1-base container.
  - User Environment:
    - Creates a non-root development user with sudo privileges
    - Uses `Zsh` as the default shell, with `Oh My Zsh` pre-installed for a modern CLI experience
  - Pre-creates the `code` folder in the home directory as a mount point for the repository.
  - Pre-creates the `catkin_ws` folder in the home directory as a mount point for the ROS workspace.

#### 1. Generate the `.env` file

Below shell will generate the `.env` file and create the `docker-compose.yaml` file.

```bash
bash docker/generate_env.sh
```

#### 2. Build the ros1-base container

```bash
bash docker/container_handler.sh build ros1-base
```

#### 3. Build the ros1-user container

```bash
bash docker/container_handler.sh build ros1-user
```
