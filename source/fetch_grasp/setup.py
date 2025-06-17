from setuptools import setup, find_packages

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    # NOTE: Add your project's dependencies here.
    # For example:
    # 'numpy>=1.20.0',
    # 'rospy',
    # 'actionlib',
    # 'control_msgs',
]

# Installation operation
setup(
    name="fetch_grasp",
    version="0.1.0",
    packages=find_packages(include=["fetch_grasp", "fetch_grasp.*"]),
    author="Jikai Wang",
    author_email="jikai.wang@utdallas.edu",
    maintainer="Jikai Wang",
    maintainer_email="jikai.wang@utdallas.edu",
    description="Fetch Grasping Toolkit",
    keywords=["fetch", "grasping", "robotics", "gazebo", "ros_noetic"],
    install_requires=INSTALL_REQUIRES,
    license="MIT",
    include_package_data=True,
    python_requires=">=3.10",
    classifiers=[
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Natural Language :: English",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    zip_safe=False,
)
