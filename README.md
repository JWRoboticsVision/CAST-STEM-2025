# CAST-STEM 2025 Summer Camp Project

[![Python](https://img.shields.io/badge/Python-3.11-3776AB.svg)](https://www.python.org/downloads/release/python-3110)
[![ROS](https://img.shields.io/badge/ROS-Melodic-22314E.svg)](http://wiki.ros.org/melodic)
[![Pytorch](https://img.shields.io/badge/Pytorch-2.1.1-EE4C2C.svg)](https://pytorch.org/)
[![Linux](https://img.shields.io/badge/OS-Ubuntu_20.04-FF9800.svg)](https://ubuntu.com/)

<!-- ![License](https://img.shields.io/badge/License-GPLv3-4E9A06.svg) -->
<!-- [![Website](https://img.shields.io/badge/Website-Visit-lightgrey.svg)](TBD) -->

This is the repository for the CAST-STEM 2025 Summer Camp project. The project aims to study perception-driven robotic grasping, where a robot first needs to recognize objects and then plans its motion for grasping.

---

## News

- **2025-06-11**:
  - added the FoundationPose installation script.
- **2025-06-10**:
  - added the Docker environment setup.

## Contents

- [CAST-STEM 2025 Summer Camp Project](#cast-stem-2025-summer-camp-project)
  - [News](#news)
  - [Contents](#contents)
  - [Prerequisites](#prerequisites)
      - [1. Git](#1-git)
      - [2. Conda Environment Manager](#2-conda-environment-manager)
      - [3. Code Editor (Visual Studio Code for example)](#3-code-editor-visual-studio-code-for-example)
  - [Environment Setup](#environment-setup)
    - [Clone the Repository](#clone-the-repository)
    - [Environment Setup (Local)](#environment-setup-local)
    - [Environment Setup (Docker)](#environment-setup-docker)
      - [1. Build the Docker Image](#1-build-the-docker-image)
      - [2. Run the Docker Container](#2-run-the-docker-container)
  - [Project Schedule](#project-schedule)
    - [Week 1: Basic Knowledge Preparation](#week-1-basic-knowledge-preparation)
      - [1. Slides](#1-slides)
      - [2. Further Readings](#2-further-readings)
      - [3. Useful Resources](#3-useful-resources)
    - [Week 2: Hands-on Practice for ROS, 6D Pose Estimation, and Fetch Gazebo](#week-2-hands-on-practice-for-ros-6d-pose-estimation-and-fetch-gazebo)
      - [1. Slides](#1-slides-1)
      - [2. Practice](#2-practice)
      - [2. Useful Resources](#2-useful-resources)

## Prerequisites

#### 1. Git

- For Linux:

```bash
sudo apt-get install git
```

- For Windows:

  - Option One: [Github Desktop](https://desktop.github.com/).
  - Option Two: [Git for Windows](https://gitforwindows.org/).

- For MacOS:

  - Option One: [Github Desktop](https://desktop.github.com/).
  - Option Two: [Homebrew](https://brew.sh/).

  ```bash
  /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
  ```

#### 2. Conda Environment Manager

Please refer to the official instruction [Installing Miniconda](https://docs.anaconda.com/miniconda/miniconda-install/) to install the miniconda.

#### 3. Code Editor (Visual Studio Code for example)

- You could install the Visual Studio Code (VSCode) from the [official website](https://code.visualstudio.com/).
- Once you have installed the VSCode, you could install below extensions:
  - [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
  - [Pylance](https://marketplace.visualstudio.com/items?itemName=ms-python.vscode-pylance)
  - [Python Debugger](https://marketplace.visualstudio.com/items?itemName=ms-python.debugpy)
  - [Jupyter](https://marketplace.visualstudio.com/items?itemName=ms-toolsai.jupyter)

## Environment Setup

### Clone the Repository

```bash
# Clone the repository
git clone https://github.com/JWRoboticsVision/CAST-STEM-2025.git

# Go to the project directory
cd CAST-STEM-2025
```

### Environment Setup (Local)

- Create the Conda Environment

```bash
conda create --prefix $PWD/.env python=3.11 -y
conda activate $PWD/.env
```

- Install the PyTorch 2.1.1

```bash
python -m pip install torch==2.1.1 torchvision==0.16.1 --index-url https://download.pytorch.org/whl/cu118 --no-cache-dir
```

- Install the dependencies

```bash
python -m pip install --no-cache-dir -r requirements.txt
```

- ~~ROS Environment Setup in Conda [Optional]~~

~~If you plan to run the ROS locally, refer to the [ROS Environment Setup](./docs/ROS_Environment_Setup.md) document for detailed steps. You can then run `roscore` to start the ROS master and debug your code under the ROS environment.~~

### Environment Setup (Docker)

If you prefer to use Docker, you can set up the environment using the provided Dockerfile. This allows you to run the project in a containerized environment.

#### 1. Build the Docker Image

Follow the instructions in the [container_installation.md](./docker/container_installation.md) to build the Docker image.

#### 2. Run the Docker Container

- Run and enter the container:

```bash
# Run the ros1-base container
bash docker/container_handler.sh run ros1-user

# Enter the container
bash docker/container_handler.sh enter ros1-user
```

- Compile the ros workspace (if you have not done so)

```bash
# Make sure you are not in the conda environment
conda deactivate
# Go to the catkin workspace
mkdir ~/catkin_ws/src && cd ~/catkin_ws/src
# Clone the fetch_ros
git clone -b ros1 https://github.com/ZebraDevs/fetch_ros.git
# Clone the fetch_gazebo
git clone -b gazebo11 https://github.com/ZebraDevs/fetch_gazebo.git
# Clone the urdf_tutorial
git clone -b ros1 https://github.com/ros/urdf_tutorial.git
# Compile the workspace
cd ~/catkin_ws && catkin_make -j$(nproc) -DPYTHON_EXECUTABLE=/usr/bin/python3
```

- Setup the Conda Environment

  - Create the conda environment in the code directory:
    The following commands will create a conda environment in the `~/code/.env` directory and activate it. This is useful for keeping the environment isolated and organized within the project directory.

  ```bash
  # Go to the code directory
  cd ~/code
  # Create and activate the conda environment
  conda create --prefix $PWD/.env python=3.11 -y
  conda activate $PWD/.env
  ```

  - Install the PyTorch 2.1.1

  ```bash
  python -m pip install torch==2.1.1 torchvision==0.16.1 --index-url https://download.pytorch.org/whl/cu118 --no-cache-dir
  ```

  - Install the dependencies

  ```bash
  python -m pip install --no-cache-dir -r requirements.txt
  ```

- Install FoundationPose:

```bash
# Install FoundationPose
bash scripts/install_foundationpose.sh

# Download checkpoint
bash scripts/download_models.sh --foundationpose
```

- Download SAM2 models:

```bash
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/sam2.1_t.pt -O ./checkpoints/sam2.1_t.pt
```

## Project Schedule

### Week 1: Basic Knowledge Preparation

#### 1. Slides

- [Pythion_Basics.ipynb](./notebooks/01_Python_Basics.ipynb)
  Introduce basics in Python, such as list, tuple, set, dictionary, class, function, loop, etc.
- [Numpy_Basics.ipynb](./notebooks/02_Python_Numpy.ipynb)
  Introduce basics in Numpy, such as array, matrix, operation, etc.
- [Pytorch_Basics.ipynb](./notebooks/06-1_Pytorch_Basics.ipynb)
  Introduce basics in Pytorch, such as tensor, operation, etc.
- [Computer_Vision_Basics.pdf](./docs/slides/01_Computer_Vision_Basics.pdf)
  - Practice 1: [CV_Transformation.ipynb](./notebooks/03-3_CV_Transformation.ipynb)
    How to apply the transformation on 3D points.
  - Practice 2: [CV_Deprojection.ipynb](./notebooks/03-1_CV_Deprojection.ipynb)
    How to depreject the 2D depth image to 3D points.
- [Introduction_to_ROS.pdf](./docs/slides/02_Introduction_to_ROS.pdf)
  Introduce the basic concepts and useful commands in ROS.
- [Introduction_to_6D_Pose_Estimation.pdf](./docs/slides/03_Introduction_to_6D_Pose_Estimation.pdf)
  Introduce the basic concepts of 6D pose estimation and FoundationPose.

#### 2. Further Readings

- [FoundationPose: Unified 6D Pose Estimation and Tracking of Novel Objects](https://arxiv.org/abs/2312.08344)
- [Segment Anything](https://arxiv.org/abs/2304.02643)
- [SAM 2: Segment Anything in Images and Videos](https://arxiv.org/pdf/2408.00714)

#### 3. Useful Resources

- Python basics https://pythonbasics.org/
- Numpy https://numpy.org/doc/stable/user/basics.html
- OpenCV https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html

### Week 2: Hands-on Practice for ROS, 6D Pose Estimation, and Fetch Gazebo

#### 1. Slides

- [06_ROS_Publish_Image.ipynb](./notebooks/06_ROS_Publish_Image.ipynb)
  Introduce how to publish images in ROS.
- [07_ROS_Subscribe_Image.ipynb](./notebooks/07_ROS_Subscribe_Image.ipynb)
  Introduce how to subscribe images in ROS.

#### 2. Practice

- Write a ROS node to publish images from the recordings.
- Write a ROS node to subscribe images and display them in a window.
- Write a ROS node to subscribe images published by the Fetch Gazebo simulation.
- Modify the FoundationPose code to:
  - detect object poses using the images published by the Fetch Gazebo simulation.
  - publish the detected poses to a ROS topic.
  - draw the detected poses on the images and publish the images to a ROS topic.

#### 2. Useful Resources

- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [ROS Python API](http://wiki.ros.org/rospy)
- ROS Messages
  - [ROS Image Message](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
  - [ROS Pose Message](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html)
  - [ROS Pose Array Message](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html)
  - [ROS TF2](http://wiki.ros.org/tf2)
- [MoveIt 1 Tutorials](https://moveit.github.io/moveit_tutorials/) for ROS Noetic
