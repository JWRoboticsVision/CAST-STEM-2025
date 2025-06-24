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
  - [Environment Setup (Docker)](#environment-setup-docker)
      - [1. Clone the Repository](#1-clone-the-repository)
      - [2. Build the Docker Image](#2-build-the-docker-image)
      - [3. Run the Docker Container](#3-run-the-docker-container)
        - [3.1 Compile the ros workspace (if you have not done so)](#31-compile-the-ros-workspace-if-you-have-not-done-so)
        - [3.2 Setup the Conda Environment](#32-setup-the-conda-environment)
  - [Project Schedule](#project-schedule)
    - [Week 1: Basic Knowledge Preparation](#week-1-basic-knowledge-preparation)
      - [1. Slides](#1-slides)
      - [2. Further Readings](#2-further-readings)
      - [3. Useful Resources](#3-useful-resources)
    - [Week 2: Hands-on Practice for ROS, 6D Pose Estimation, and Fetch Gazebo](#week-2-hands-on-practice-for-ros-6d-pose-estimation-and-fetch-gazebo)
      - [1. Slides](#1-slides-1)
      - [2. Practice](#2-practice)
      - [2. Useful Resources](#2-useful-resources)
    - [Week 3: Fetch Gazebo Simulation](#week-3-fetch-gazebo-simulation)
      - [1. Fetch Grasping Demo](#1-fetch-grasping-demo)
      - [2. SceneReplica Reimplementation](#2-scenereplica-reimplementation)
    - [Week 4: Integrate FoundationPose, NIDS-Net with Fetch Gazebo Simulation](#week-4-integrate-foundationpose-nids-net-with-fetch-gazebo-simulation)
      - [1. Prerequisites](#1-prerequisites)
      - [2. Get RGBD Images and CameraInfo from ROS topics](#2-get-rgbd-images-and-camerainfo-from-ros-topics)
      - [3. Run FoundationPose on the Saved RGBD Images](#3-run-foundationpose-on-the-saved-rgbd-images)

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

#### 2. Conda Environment Manager

Please refer to the official instruction [Installing Miniconda](https://docs.anaconda.com/miniconda/miniconda-install/) to install the miniconda.

#### 3. Code Editor (Visual Studio Code for example)

- You could install the Visual Studio Code (VSCode) from the [official website](https://code.visualstudio.com/).
- Once you have installed the VSCode, you could install below extensions:
  - [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
  - [Pylance](https://marketplace.visualstudio.com/items?itemName=ms-python.vscode-pylance)
  - [Python Debugger](https://marketplace.visualstudio.com/items?itemName=ms-python.debugpy)
  - [Jupyter](https://marketplace.visualstudio.com/items?itemName=ms-toolsai.jupyter)

## Environment Setup (Docker)

If you prefer to use Docker, you can set up the environment using the provided Dockerfile. This allows you to run the project in a containerized environment.

#### 1. Clone the Repository

```bash
# Clone the repository
git clone https://github.com/JWRoboticsVision/CAST-STEM-2025.git

# Go to the project directory
cd CAST-STEM-2025
```

#### 2. Build the Docker Image

Follow below instructions to build the Docker image based on your operating system.

- For Linux, follow the [Linux Installation Guide](./docs/container_installation_linux.md).
- For Windows, follow the [Windows Installation Guide](./docs/container_installation_windows.md).

#### 3. Run the Docker Container

- Run and enter the container:

```bash
# Run the ros1-base container
bash ./docker/container_handler.sh run ros1-user

# Enter the container
bash ./docker/container_handler.sh enter ros1-user
```

##### 3.1 Compile the ros workspace (if you have not done so)

```bash
# Make sure you are not in the conda environment
conda deactivate
# Update rosdeps
rosdep update --rosdistro=$ROS_DISTRO
# Go to the catkin workspace
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
# Fetch Robot ROS package
git clone -b ros1 https://github.com/ZebraDevs/fetch_ros.git
# Fetch Gazebo Simulator
git clone -b gazebo11 https://github.com/ZebraDevs/fetch_gazebo.git
# Clone the urdf_tutorial
git clone -b ros1 https://github.com/ros/urdf_tutorial.git
# Compile the workspace
cd ~/catkin_ws && catkin_make -j$(nproc) -DPYTHON_EXECUTABLE=/usr/bin/python3
# Source the workspace
source ~/catkin_ws/devel/setup.zsh
# Verify the workspace
roslaunch urdf_tutorial display.launch  model:=${HOME}/catkin_ws/src/fetch_ros/fetch_description/robots/fetch.urdf
```

##### 3.2 Setup the Conda Environment

- Create the conda environment in the code directory:
  The following commands will create a conda environment in the `~/code/.env` directory and activate it. This is useful for keeping the environment isolated and organized within the project directory.

```bash
# Go to the code directory
cd ~/code
# Create and activate the conda environment
conda create --prefix $PWD/.env python=3.11 libffi=3.4 pyside2=5.15 -y
conda activate $PWD/.env
```

- Install the PyTorch 2.1.1

```bash
python -m pip install torch==2.1.1 torchvision==0.16.1 --index-url https://download.pytorch.org/whl/cu118 --no-cache-dir
```

- Install the dependencies

```bash
python -m pip install -r requirements.txt --no-cache-dir
```

- Install the fetch_grasp package

```bash
python -m pip install -e source/fetch_grasp --no-cache-dir
```

- Install FoundationPose:

```bash
# Install FoundationPose
bash scripts/install_foundationpose.sh
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

### Week 3: Fetch Gazebo Simulation

In this week, we will focus on the Fetch Gazebo simulation. The goal is to use the Fetch robot in the Gazebo simulation environment to perform grasping tasks.

#### 1. Fetch Grasping Demo

Download and unzip the `my_demo.zip` file from [box](https://utdallas.box.com/s/puvnx931jzwk4o2bqrv5fnqnojzu80ar) and place it under the `./docker/ros/catkin_ws/src` directory.

- Run Docker container:

```bash
bash ./docker/container_handler.sh run ros1-user
```

- Enter the container:

```bash
bash ./docker/container_handler.sh enter ros1-user
```

- **Compile** the ROS workspace:

```bash
# Go to the catkin workspace
cd ~/catkin_ws && catkin_make -j$(nproc) -DPYTHON_EXECUTABLE=/usr/bin/python3
# Source the workspace
source ~/catkin_ws/devel/setup.zsh
```

- Link the models of my_demo to the Gazebo model path:

To load the models properly in Gazebo, we need to link the `my_demo` models to the Gazebo model path. This allows Gazebo to find the models when launching the simulation.

```bash
cd ~/.gazebo && ln -s ~/catkin_ws/src/my_demo/models models
```

- **Terminal 1:** Start the ROS master

```bash
roscore
```

![fetch_demo_roscore](./docs/resources/fetch_demo_roscore.png)

- **Terminal 2:** Start the Gazebo Simulation

Below launch file will start the Gazebo simulation environment with a simple scene containing a Fetch robot, a table and the YCB Cracker object.

```bash
roslaunch my_demo table_ycb.launch
```

![fetch_demo_moveit_gazebo](./docs/resources/fetch_demo_moveit_gazebo.png)

- **Terminal 3:** Launch the MoveIt Planning Interface

```bash
roslaunch fetch_moveit_config move_group.launch
```

![fetch_demo_moveit_planning](./docs/resources/fetch_demo_moveit_planning.png)

- **Terminal 4:** Start Rviz

The `fetch_gazebo.rviz` will display the color and depth images from the Fetch robot's cameras, the robot model, and the MoveIt PlanningScene.

```bash
rviz -d ${HOME}/code/config/rviz/fetch_gazebo.rviz
```

![fetch_demo_moveit_rviz](./docs/resources/fetch_demo_moveit_rviz.png)

- **Terminal 5:** Run the Grasping

The `grasp_cracker.py` script will execute below tasks:

- Lift the Fetch robot's torso.
- Setup the PlanningScene
- Adjust the Fetch robot's camera to look at the tabletop.
- Get the cracker 6D pose directly from the Gazebo simulation.
- Load the grasp data for the cracker object.
- Plan the grasping motion using MoveIt.
  - First sort the grasps based on the distance to the gripper.
  - Then plan the grasping motion for each grasp, until a valid grasp is found.
- Execute the grasping motion.

```bash
cd ~/catkin_ws/src/my_demo/scripts && python grasp_cracker.py
```

![fetch_demo_moveit_grasping](./docs/resources/fetch_demo_moveit_grasping.png)

#### 2. SceneReplica Reimplementation

In this section, we will reimplement the [SceneReplica](https://github.com/IRVLUTD/SceneReplica) benchmarking using the Fetch robot in the Gazebo simulation environment.

1. Download the modified SceneReplica from [box](https://utdallas.box.com/s/0ueu09jzm0j9i9uk6ppfwv28ad7rw1hn).
2. Unzip the downloaded file and place it in the `third-party` directory.
3. Data Setup:
   Follow the instructions in the [SceneReplica Data Setup](https://github.com/IRVLUTD/SceneReplica/tree/main#data-setup) to set up the data for SceneReplica.

```
Datasets
   |--benchmarking
      |--models/
      |--grasp_data
         |--refined_grasps
            |-- fetch_gripper-{object_name}.json
         |--sgrasps.pk
      |--final_scenes
         |--scene_data/
            |-- scene_id_*.pk scene pickle files
         |--metadata/
            |-- meta-00*.mat metadata .mat files
            |-- color-00*.png color images for scene
            |-- depth-00*.png depth images for scene
         |--scene_ids.txt : selected scene ids on each line
```

4. Run the SceneReplica benchmarking in the Gazebo simulation environment using the Docker container.

- Run and Enter the Docker container:

```bash
bash docker/container_handler.sh run ros1-user
bash docker/container_handler.sh enter ros1-user
```

- Link the SceneReplica models to the Gazebo model path:

```bash
cd ~/.gazebo && rm models && ln -s ~/code/third-party/SceneReplica/Datasets/benchmarking/models
```

- **Terminal 1:** Start the ROS master (if not already running)

```bash
roscore
```

- **Terminal 2:** Start the Fetch Gazebo simulation with Just Robot

```bash
roslaunch ~/code/third-party/SceneReplica/launch/just_robot.launch
```

- **Terminal 3:** Start the MoveIt Planning Interface

```bash
roslaunch ~/code/third-party/SceneReplica/launch/moveit_sim.launch
```

- **Terminal 4:** Start Rviz

```bash
rviz -d ~/code/config/rviz/grasp_sim.rviz
```

- **Terminal 5:** Setup the desired scene in Gazebo
  Available scene ids: 10, 25, 27, 33, 36, 38, 39, 48, 56, 65, 68, 77, 83, 84, 104, 122, 130, 141, 148, 161

```bash
cd ~/code/third-party/SceneReplica/src && python setup_scene_sim.py --data_dir ../Datasets/benchmarking
# Select the scene id you want to setup
# For example, to setup scene id 10
```

- **Terminal 6:** Run the Model-based Grasping

```bash
# Go to the SceneReplica source directory
cd ~/code/third-party/SceneReplica/src && python bench_model_based_grasping.py \
  --pose_method gazebo \
  --obj_order nearest_first \
  --data_dir ../Datasets/benchmarking \
  --scene_idx 10
```

### Week 4: Integrate FoundationPose, NIDS-Net with Fetch Gazebo Simulation

In this week, we will integrate the FoundationPose and NIDS-Net with the Fetch Gazebo simulation environment.

#### 1. Prerequisites

Repeat steps till Terminal 3 as described in the [Fetch Gazebo Simulation](#2-scenereplica-reimplementation) section to set up the Fetch Gazebo simulation environment for Scene id **10**.

#### 2. Get RGBD Images and CameraInfo from ROS topics

In this practice, we will subscribe to the RGBD images and CameraInfo published by the Fetch Gazebo simulation and save them under `/datasets/tmp` for later use.

- Practice 1: get the color, depth images and cam_K from simulation.

Complete the code in `/notebooks/08_FoundationPoseROS.py` to get the RGBD images and CameraInfo from the subscribed ROS topics published by the Fetch Gazebo simulation. The answer could be found [here](notebooks/08_FoundationPoseROS_answer.py).

| Color Image                                | Depth Image                                    |
| ------------------------------------------ | ---------------------------------------------- |
| ![color](./docs/resources/color_image.png) | ![depth](./docs/resources/depth_image_vis.png) |

- Practice 2: Use SAM2 based segmentation toolkit to get the segment mask for **Power Drill**.

```bash
python tools/01_run_sam2_segmentation.py
```

The segmentation results will be saved under `/datasets/tmp`.

![mask](./docs/resources/mask_image_vis.png)

#### 3. Run FoundationPose on the Saved RGBD Images

Ensure you have installed the FoundationPose as described in the [Environment Setup](#environment-setup-docker) section.

Now, we have the inputs ready, we can run the FoundationPose to get the 6D poses of the objects in the scene.

- **Notebook [05_FoundationPoseWrapper.ipynb](./notebooks/05_FoundationPoseWrapper.ipynb)**: Understand how to run FoundationPose to estimate the 6D poses of the objects in the scene. The notebook will guide you through the process of preparing the inputs, running the FoundationPose, and visualizing the results.

<!--
#### 4. NIDS-Net: Run Segmentation on the Published RGB Image

TBD -->
