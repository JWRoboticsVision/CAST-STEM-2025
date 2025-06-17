#!/usr/bin/env python

"""
CS 6301 Homework 3 Programming
Planning scene and IK
"""

import sys
import time
import rospy
import roslib
import tf
import numpy as np
import moveit_commander

from transforms3d.quaternions import mat2quat, quat2mat
from geometry_msgs.msg import PoseStamped
from trac_ik_python.trac_ik import IK

roslib.load_manifest('gazebo_msgs')
from gazebo_msgs.srv import GetModelState


def ros_quat(tf_quat): #wxyz -> xyzw
    quat = np.zeros(4)
    quat[-1] = tf_quat[0]
    quat[:-1] = tf_quat[1:]
    return quat


# Convert quaternion and translation to a 4x4 tranformation matrix
# See Appendix B.3 in Lynch and Park, Modern Robotics for the definition of quaternion
def ros_qt_to_rt(rot, trans):
    qt = np.zeros((4,), dtype=np.float32)
    qt[0] = rot[3]
    qt[1] = rot[0]
    qt[2] = rot[1]
    qt[3] = rot[2]
    obj_T = np.eye(4)
    obj_T[:3, :3] = quat2mat(qt)
    obj_T[:3, 3] = trans

    return obj_T


# Convert a ROS pose message to a 4x4 tranformation matrix
def ros_pose_to_rt(pose):
    qarray = [0, 0, 0, 0]
    qarray[0] = pose.orientation.x
    qarray[1] = pose.orientation.y
    qarray[2] = pose.orientation.z
    qarray[3] = pose.orientation.w

    t = [0, 0, 0]
    t[0] = pose.position.x
    t[1] = pose.position.y
    t[2] = pose.position.z

    return ros_qt_to_rt(qarray, t)


# Query pose of frames from the Gazebo environment
def get_pose_gazebo(model_name, relative_entity_name=''):

    def gms_client(model_name, relative_entity_name):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp1 = gms(model_name, relative_entity_name)
            return resp1
        except (rospy.ServiceException, e):
            print("Service call failed: %s" % e)

    # query the object pose in Gazebo world T_wo
    res = gms_client(model_name, relative_entity_name)
    T_wo = ros_pose_to_rt(res.pose)

    # query fetch base link pose in Gazebo world T_wb
    res = gms_client(model_name='fetch', relative_entity_name='base_link')
    T_wb = ros_pose_to_rt(res.pose)

    # compute the object pose in robot base link T_bo
    T_bo = np.matmul(np.linalg.inv(T_wb), T_wo)
    return T_bo


if __name__ == "__main__":
    """
    Main function to run the code
    """

    # intialize ros node
    rospy.init_node('planning_scene_block')

    # query the demo cube pose
    model_name = 'demo_cube'
    T = get_pose_gazebo(model_name)
    print('pose of the demo cube')
    print(T)

    # translation
    trans = T[:3, 3]
    # quaternion in ros
    qt = ros_quat(mat2quat(T[:3, :3]))


    # --------- initialize moveit components ------
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander('arm')
    # planning scene
    scene = moveit_commander.PlanningSceneInterface()
    scene.clear()
    robot = moveit_commander.RobotCommander()

    # print information about the planner
    planning_frame = group.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    joint_state = robot.get_current_state().joint_state
    for i in range(len(joint_state.name)):
        print(joint_state.name[i], joint_state.position[i])

    # add objects into the planning scene
    rospy.sleep(1.0)
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    # add a box for robot base to avoid hitting the base
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0.18
    scene.add_box("base", p, (0.56, 0.56, 0.4))

    # add a box for the demo cube
    p.pose.position.x = trans[0]
    p.pose.position.y = trans[1]
    p.pose.position.z = trans[2]
    scene.add_box("demo_cube", p, (0.06, 0.06, 0.06))

    # get the current pose of the gripper
    efpose = group.get_current_pose().pose
    position = efpose.position
    orientation = efpose.orientation
    print('end-effector pose of the robot')
    print(efpose)

    # get the current joints
    joints = group.get_current_joint_values()
    print('current joint state of the robot')
    print(group.get_active_joints())
    print(joints)

    # define the IK solver from track_ik
    ik_solver = IK("base_link", "wrist_roll_link")

    # change the joint limit of torso_lift_joint in order to fix the torso lift
    lower_bound, upper_bound = ik_solver.get_joint_limits()
    lower_bound = list(lower_bound)
    upper_bound = list(upper_bound)
    lower_bound[0] = 0
    upper_bound[0] = 0
    ik_solver.set_joint_limits(lower_bound, upper_bound)

    # use initial seed as zeros
    seed_state = [0.0] * ik_solver.number_of_joints
    # seed_state = joints # + np.random.randn(ik_solver.number_of_joints) * 0.01

    sol = ik_solver.get_ik(seed_state,
                position.x, position.y, position.z,  # X, Y, Z
                orientation.x, orientation.y, orientation.z, orientation.w)  # QX, QY, QZ, QW
    print('Solution from IK:')
    print(ik_solver.joint_names)
    print(sol)
