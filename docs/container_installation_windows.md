# Local Desktop Environment Setup

## Contents

- [Local Desktop Environment Setup](#local-desktop-environment-setup)
  - [Contents](#contents)
  - [Prerequisites](#prerequisites)
    - [1. Install Windows Terminal](#1-install-windows-terminal)
    - [2. Install the Docker Desktop Application](#2-install-the-docker-desktop-application)
    - [3. Enable WSL 2 and Docker WSL 2 backend (if not already enabled)](#3-enable-wsl-2-and-docker-wsl-2-backend-if-not-already-enabled)
    - [4. Install Ubuntu 20.04 LTS from the Microsoft Store](#4-install-ubuntu-2004-lts-from-the-microsoft-store)
    - [5. Enable Ubuntu 20.04 LTS in Docker Desktop](#5-enable-ubuntu-2004-lts-in-docker-desktop)
    - [6. Configuration in Ubuntu WSL 2](#6-configuration-in-ubuntu-wsl-2)
    - [7. Build the Docker Container](#7-build-the-docker-container)
      - [7.1. Generate the `.env` and `docker-compose.yaml` files](#71-generate-the-env-and-docker-composeyaml-files)
      - [7.2 Build the ROS 1 Image](#72-build-the-ros-1-image)

## Prerequisites

- Windows 11
- Docker Desktop installed
- Windows Subsystem for Linux (WSL) 2 enabled
- Ubuntu 20.04 LTS installed from the Microsoft Store
- Windows Terminal installed from the Microsoft Store

### 1. Install Windows Terminal

Download and install Windows Terminal from the [Microsoft Store](https://aka.ms/terminal).

### 2. Install the Docker Desktop Application

Download and install Docker Desktop from the [official website](https://www.docker.com/products/docker-desktop/).

### 3. Enable WSL 2 and Docker WSL 2 backend (if not already enabled)

The WSL 2 backend should be enabled when installing Docker Desktop. If you have not done this during the installation, you can enable it following the instructions on the [Microsoft documentation](https://docs.microsoft.com/en-us/windows/wsl/install).
Then, open `Docker Desktop settings`, go to the `General` tab, and check the box for `Use the WSL 2 based engine`.

This will allow Docker to run using the Windows Subsystem for Linux (WSL) 2, which is required for running Docker containers on Windows.

![windows_docker_setting-1](/docs/resources/windows-docker-setting-1.png)

### 4. Install Ubuntu 20.04 LTS from the Microsoft Store

Install `Ubuntu 20.04 LTS` from the [Microsoft Store](https://apps.microsoft.com/detail/9mttcl66cpxj?ocid=webpdpshare).

### 5. Enable Ubuntu 20.04 LTS in Docker Desktop

Open Docker Desktop settings, go to the "Resources" tab, and then to the "WSL Integration" section. Enable the Ubuntu 20.04 LTS distribution by checking the box next to it.

![windows_docker_setting-2](/docs/resources/windows-docker-setting-2.png)

### 6. Configuration in Ubuntu WSL 2

Suppose you have setup the user/password for Ubuntu 20.04 LTS properly.
Open the Ubuntu 20.04 LTS terminal from Windows Terminal and run the following commands to configure your environment:

- Update the package list and install necessary packages

```bash
sudo apt update
sudo apt install -y git curl wget build-essential
```

- Mount the Windows folder to the WSL filesystem

Below code will use the default mount point for WSL, which is `/mnt/c` for the C: drive. If you want to mount a different drive, change the path accordingly.

```bash
# Create a symbolic link to the C: drive in your home directory
ln -s /mnt/c ~/c_drive

# Create a directory for your projects
mkdir -p ~/c_drive/MyProjects
# Change to the projects directory
cd ~/c_drive/MyProjects
# Clone the repository
git clone https://github.com/JWRoboticsVision/CAST-STEM-2025.git
```

### 7. Build the Docker Container

#### 7.1. Generate the `.env` and `docker-compose.yaml` files

Below shell will generate the `.env` file, create the binding directories, and create the `docker-compose.yaml` file.

```bash
bash docker/generate_env.sh
```

#### 7.2 Build the ROS 1 Image

- Build the image `irvlutd/ubuntu20.04-cuda-11.8.0-ros-noetic:latest`:

```bash
bash docker/container_handler.sh build ros1-user
```

- Ensure the image is built successfully by running:

```bash
docker images
# You should see the image `irvlutd/ubuntu20.04-cuda-11.8.0-ros-noetic` in the list
```
