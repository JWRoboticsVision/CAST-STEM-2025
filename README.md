# CAST-STEM 2025 Summer Camp Project

This is the repository for the CAST-STEM 2025 Summer Camp project. The project aims to study perception-driven robotic grasping, where a robot first needs to recognize objects and then plans its motion for grasping.

---

## Contents

- [CAST-STEM 2025 Summer Camp Project](#cast-stem-2025-summer-camp-project)
  - [Contents](#contents)
  - [Prerequisites](#prerequisites)
      - [1. Git](#1-git)
      - [2. Conda Environment Manager](#2-conda-environment-manager)
      - [3. Code Editor (Visual Studio Code for example)](#3-code-editor-visual-studio-code-for-example)
  - [Environment Setup](#environment-setup)
  - [Project Schedule](#project-schedule)
    - [Week 1: Basic Knowledge Preparation](#week-1-basic-knowledge-preparation)
      - [1. Slides](#1-slides)
      - [2. Further Readings](#2-further-readings)
      - [3. Useful Resources](#3-useful-resources)

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

1. Create the Conda Environment

```bash
conda create --name summer_camp python=3.11
conda activate summer_camp
```

2. Clone the Repository

```bash
git clone --recursive https://github.com/JWRoboticsVision/CAST-STEM-2025.git
cd CAST-STEM-2024
```

3. Install Dependencies

```bash
python -m pip install --no-cache-dir -r requirements.txt
```

1. ROS Environment Setup [Optional]

If you plan to run the ROS locally, refer to the [ROS Environment Setup](./docs/ROS_Environment_Setup.md) document for detailed steps. You can then run `roscore` to start the ROS master and debug your code under the ROS environment.

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
