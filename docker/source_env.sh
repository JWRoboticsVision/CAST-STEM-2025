source /opt/ros/noetic/setup.zsh
source ~/catkin_ws/devel/setup.zsh

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')
