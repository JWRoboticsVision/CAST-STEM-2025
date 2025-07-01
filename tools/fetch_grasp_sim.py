import copy
import sys
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.srv import GetModelState
import moveit_commander
import moveit_msgs.msg
import tf2_ros
from tf.transformations import quaternion_matrix

from fetch_grasp.grippers import FetchGripper
from fetch_grasp.utils import PROJECT_ROOT
from fetch_grasp.utils.scene import ObjectService, SceneMaker
from fetch_grasp.utils.control import PointHeadClient, FollowTrajectoryClient
from fetch_grasp.utils.grasp import (
    parse_grasps,
    sort_and_filter_grasps,
    model_based_top_down_grasp,
    get_object_verts,
    get_standoff_wp_poses,
    lift_arm_cartesian,
    move_arm_to_dropoff,
    rotate_gripper,
)
from fetch_grasp.utils.ros import ros_pose_to_rt, rt_to_ros_qt, rt_to_ros_pose
from fetch_grasp.utils.stow_or_tuck_arm import reset_arm_stow

Z_OFFSET = -0.03  # difference between Real World and Gazebo table
TABLE_HEIGHT = 0.78 + Z_OFFSET

MODEL_NAMES = [
    "003_cracker_box",
    # "005_tomato_soup_can",
    # "006_mustard_bottle",
    # "007_tuna_fish_can",
    # "008_pudding_box",
    # "009_gelatin_box",
    # "010_potted_meat_can",
    # "011_banana",
    # "021_bleach_cleanser",
    # "024_bowl",
    # "025_mug",
    "035_power_drill",
    # "037_scissors",
    # "040_large_marker",
    # "052_extra_large_clamp",
]


def get_tf_pose(tf_buffer, target_frame, base_frame=None):
    try:
        transform = tf_buffer.lookup_transform(
            base_frame, target_frame, rospy.Time.now(), rospy.Duration(1.0)
        ).transform
        quat = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        RT_obj = quaternion_matrix(quat)
        RT_obj[:3, 3] = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ):
        RT_obj = None
    return RT_obj


def plan_grasp(group, scene, RT_grasps_base, grasp_index, obj_name, RT_obj, fraction_thresh=0.9):
    """
    RT_grasps_base is with shape (50, 4, 4): 50 grasps in the robot base frame
    The plan_grasp function tries to plan a trajectory to each grasp. It stops when a plan is found.
    A standoff is a gripper pose with a short distance along x-axis of the gripper frame before grasping the object.
    """
    # number of grasps
    n = RT_grasps_base.shape[0]
    n = RT_grasps_base.shape[0]
    pose_standoff = get_standoff_wp_poses()
    flag_plan = False

    # for each grasp
    for i in range(n):
        RT_grasp = RT_grasps_base[i]
        grasp_idx = grasp_index[i]
        print(f"{i}: Planning for {obj_name} with grasp_idx {grasp_idx}")

        standoff_grasp_global = np.matmul(RT_grasp, pose_standoff)

        group.stop()
        group.clear_pose_targets()

        # plan to the standoff
        quat, trans = rt_to_ros_qt(standoff_grasp_global[0, :, :])  # xyzw for quat
        pose_goal = Pose()
        pose_goal.orientation.w = quat[3]
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.position.x = trans[0]
        pose_goal.position.y = trans[1]
        pose_goal.position.z = trans[2]
        group.set_pose_target(pose_goal)
        plan = group.plan()

        if plan[0]:
            trajectory = plan[1]
            flag_plan = True
            scene.remove_world_object(obj_name)
            waypoints = []
            wpose = group.get_current_pose().pose
            for i in range(1, standoff_grasp_global.shape[0]):
                wpose = rt_to_ros_pose(wpose, standoff_grasp_global[i])
                waypoints.append(copy.deepcopy(wpose))

            (plan_standoff, fraction) = group.compute_cartesian_path(waypoints, 0.01, True)  # eef_step
            print(f"found a plan for grasp, fraction: {fraction}")
            if fraction >= fraction_thresh:
                print("Found FULL PLAN!")
                return RT_grasp, grasp_idx, trajectory, plan_standoff
            else:
                obj_mesh_path = models_dir / obj_name / "textured_simple.obj"
                p = PoseStamped()
                p.pose = rt_to_ros_pose(p.pose, RT_obj)
                scene.add_mesh(obj_name, p, f"{obj_mesh_path}")
        else:
            print(f"no plan for grasp {i} with index {grasp_idx}")
    if not flag_plan:
        print("no plan found in plan_grasp()")
    return None, -1, None, None


def get_pose(object_name: str, pose_method="gazebo"):
    """
    Calls the suitable function depending on pose method

    Input:
    - object_name (str) : name of the object mode, e.g '003_cracker_box'
    - pose_method (str) : name of the model based pose method, e.g. 'poserbpf'
        - options : {'gazebo', 'posecnn', 'poserbpf'}

    Returns:
    - RT_object (4,4 np.ndarray) : 4x4 transform of the object in robot's base_link frame
    """
    if pose_method == "gazebo":
        return get_pose_gazebo(object_name)
    elif pose_method == "fdpose":
        return get_pose_fdpose(object_name)


def get_pose_gazebo(model_name, relative_entity_name=""):
    # Query pose of frames from the Gazebo environment

    def gms_client(model_name, relative_entity_name):
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            resp1 = gms(model_name, relative_entity_name)
            return resp1
        except (rospy.ServiceException, e):
            print("Service call failed: %s" % e)

    # query the object pose in Gazebo world T_wo
    res = gms_client(model_name, relative_entity_name)
    RT_obj = ros_pose_to_rt(res.pose)

    # query fetch base link
    res = gms_client(model_name="fetch", relative_entity_name="base_link")
    RT_base = ros_pose_to_rt(res.pose)

    # object pose in robot base
    RT = np.matmul(np.linalg.inv(RT_base), RT_obj)
    return RT


def get_pose_fdpose(object_name):
    pass


def grasp_with_rt(gripper, group, scene, object_name, display_trajectory_publisher, RT_grasp):
    """
    A method the included the actions of pushing and sweeping according to direction.
    It first set its arm to the left/right of the given location, then sweeps to the left/right of the cube to achieve
    the pushing motion
    :param group: the moveit group of joints
    :param display_trajectory_publisher: for visualization of the planned trajectory
    :param RT_grasp: gripper pose for grasping
    :return:
    """
    pose_standoff = get_standoff_wp_poses()
    standoff_grasp_global = np.matmul(RT_grasp, pose_standoff)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    # plan to the standoff
    quat, trans = rt_to_ros_qt(standoff_grasp_global[0, :, :])  # xyzw for quat
    pose_goal = Pose()
    pose_goal.orientation.w = quat[3]
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.position.x = trans[0]
    pose_goal.position.y = trans[1]
    pose_goal.position.z = trans[2]
    group.set_pose_target(pose_goal)
    plan = group.plan()
    trajectory = plan[1]
    if not plan[0]:
        print("no plan found in grasp()")
        return

    # visualize plan
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(trajectory)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    input("execute?")
    group.execute(trajectory, wait=True)
    group.stop()
    group.clear_pose_targets()

    # remove the target from the planning scene for grasping
    scene.remove_world_object(object_name)

    waypoints = []
    wpose = group.get_current_pose().pose
    for i in range(1, standoff_grasp_global.shape[0]):
        wpose = rt_to_ros_pose(wpose, standoff_grasp_global[i])
        waypoints.append(copy.deepcopy(wpose))
    (plan_standoff, fraction) = group.compute_cartesian_path(
        waypoints, 0.01, True  # waypoints to follow  # eef_step
    )  # jump_threshold

    print(f"{object_name}: FRACTION: {fraction}")
    trajectory = plan_standoff

    # visualize plan
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(trajectory)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    input("execute?")
    group.execute(trajectory, wait=True)
    group.stop()
    group.clear_pose_targets()

    # close gripper
    print("close gripper")
    gripper.close()
    rospy.sleep(2)


def get_gripper_rt(tf_buffer):
    transform = tf_buffer.lookup_transform(
        "base_link", "wrist_roll_link", rospy.Time.now(), rospy.Duration(1.0)
    ).transform
    quat = [
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ]
    RT_gripper = quaternion_matrix(quat)
    RT_gripper[:3, 3] = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    return RT_gripper


def setup_planning_scene(scene, object_names, pose_method="gazebo", table_position=(0.8, 0, 0)):
    """Sets the Motion Planning scene for MoveIt."""
    # Clear Planning Scene
    scene.clear()
    rospy.sleep(1.0)
    scene.remove_world_object()
    rospy.sleep(1.0)

    # create PoseStamped msg
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    # -------- planning scene set-up -------
    rospy.loginfo("adding table object")
    p.pose.position.x = table_position[0]
    p.pose.position.y = 0
    p.pose.position.z = TABLE_HEIGHT - 0.5  # 0.5 = half length of moveit obstacle box
    scene.add_box("table", p, (1, 2, 1))
    # add a box for robot base
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0.18
    scene.add_box("base", p, (0.56, 0.56, 0.4))
    # add objects
    for obj_name in object_names:
        RT_obj = get_pose(obj_name, pose_method)
        p.pose = rt_to_ros_pose(p.pose, RT_obj)
        obj_mesh_path = models_dir / obj_name / "textured_simple.obj"
        scene.add_mesh(obj_name, p, f"{obj_mesh_path}")


def create_scene():
    rospy.loginfo("=" * 5 + " Creating scene in Gazebo... " + "=" * 5)
    # ---------- Add objects to the scene ----------
    sample_scene, _ = scene_m.create_scene()
    object_names = sorted(sample_scene.keys())

    cleanup_scene(object_names)

    rospy.loginfo("Adding the cafe table...")
    objs.add_object("cafe_table_org", [*table_position, 0, 0, 0, 1])

    rospy.loginfo(f"Adding objects...")
    for obj_name in object_names:
        objs.add_object(obj_name, sample_scene[obj_name])
    rospy.sleep(3.0)  # wait for objects to be added
    print("Objects added to the scene!\n" + f"{object_names}")

    return sample_scene, object_names


def cleanup_scene(scene_objects):
    rospy.loginfo("Cleaning up the scene...")
    all_objects = scene_objects + ["cafe_table_org"]
    for obj in all_objects:
        try:
            objs.delete_object(obj)
        except Exception as e:
            pass


def do_motion_planning():
    # ---------- Setup the planning scene ----------
    rospy.loginfo("Setting up the planning scene...")
    for obj_idx, obj_name in enumerate(object_names):
        print("=" * 5 + f" Start Motion Planning for {obj_name} " + "=" * 5)
        grasp_num, trajectory_standoff, trajectory_final = None, None, None
        gripper.open()
        setup_planning_scene(scene, object_names, pose_method)

        # get the object pose in robot base frame
        RT_obj = get_pose(obj_name, pose_method)

        direct_topdown = False
        if not direct_topdown:
            # Current gripper pose
            RT_gripper = get_gripper_rt(tf_buffer)
            print("RT_gripper", RT_gripper)
            # Using Graspit generated grasp to test the Pose Detection (model based grasping)

            # Load grasps
            grasp_file = grasp_dir / f"fetch_gripper-{obj_name}.json"
            RT_grasps = parse_grasps(grasp_file)

            # Sort grasps according to distances to gripper
            # all the grasps in the robot base frame
            RT_grasps_base, grasp_index, _ = sort_and_filter_grasps(RT_obj, RT_gripper, RT_grasps, TABLE_HEIGHT)
            print(f"grasp index {grasp_index}")
            if grasp_index is None:
                direct_topdown = True
            else:
                # grasp planning
                RT_grasp, grasp_num, trajectory_standoff, trajectory_final = plan_grasp(
                    group, scene, RT_grasps_base, grasp_index, obj_name, RT_obj
                )

        if direct_topdown or (grasp_num == -1 or (not trajectory_standoff) or (not trajectory_final)):
            rospy.logwarn("No plans found for direct grasping, trying TOP-DOWN!")
            mesh_p = models_dir / obj_name / "textured_simple.obj"
            obj_pts = get_object_verts(mesh_p, pose=RT_obj)
            RT_grasp, g_width = model_based_top_down_grasp(obj_pts)
            print(f"Gripper Width: {g_width}")
            if obj_name in {"052_extra_large_clamp", "025_mug"}:
                g_width = -1
            if g_width < (0.1 - 0.002):
                grasp_with_rt(gripper, group, scene, obj_name, display_trajectory_publisher, RT_grasp)
                rospy.loginfo("Grasping with TOP-DOWN grasp!")
            else:
                rospy.logwarn("TOP DOWN FAILED!! Object too wide.")
        elif trajectory_standoff and trajectory_final and grasp_num != -1:
            grasp_with_rt(
                gripper,
                group,
                scene,
                obj_name,
                display_trajectory_publisher,
                RT_grasp,
            )
        rospy.loginfo(f"{obj_name} reached successfully!")

        # ------------------------ LIFTING OBJECT --------------------------#
        if gripper.is_fully_closed() or gripper.is_fully_open():
            print("Gripper fully open/closed (after Grasping)....Not Lifting!")
            # TODO: LOG Grasping failure to log file with scene_id, object name, pose method, and order (all exp params)
            rospy.loginfo(
                f"Gripper fully open/closed (after Grasping) object_name--{obj_name} pose_method--{pose_method}"
            )
        else:
            print("Trying to lift object")
            RT_gripper = get_gripper_rt(tf_buffer)
            print("RT_gripper before lifting", RT_gripper)
            lift_arm_cartesian(group, RT_gripper)

            # ----------------------- MOVING OBJECT ------------------------#
            if gripper.is_fully_closed() or gripper.is_fully_open():
                print("Gripper fully open/closed (after Lifting)....Not Moving!")
            else:
                print(f"{obj_name} successfully lifted")
                print("Trying to move object")
                RT_gripper = get_gripper_rt(tf_buffer)
                rotate_gripper(group, RT_gripper)
                RT_gripper = get_gripper_rt(tf_buffer)
                print("RT_gripper after lift", RT_gripper)
                move_arm_to_dropoff(group, RT_gripper, x_final=0.78)
                if gripper.is_fully_closed() or gripper.is_fully_open():
                    print("Gripper fully open/closed (after Moving)....")
                print(f"{obj_name} successfully droppedoff")

        # ------------------------ OPEN GRIPPER & STOW ---------------------#
        input("Open Gripper??")
        gripper.open()
        print("STOWING THE GRIPPER")
        reset_arm_stow(group)

        # Clear Planning Scene
        scene.clear()
        rospy.sleep(1.0)
        scene.remove_world_object()

        for i in range(3):
            x = input("proceed next ?").lower()
            if x == "n":
                sys.exit(1)


if __name__ == "__main__":
    pose_method = "gazebo"  # gazebo, fdpose
    models_dir = PROJECT_ROOT / "datasets/models"
    grasp_dir = PROJECT_ROOT / "datasets/grasp_data/refined_grasps"
    stable_pose_file = PROJECT_ROOT / "datasets/pose_data/selected_poses.pk"
    grid_r = 2
    grid_c = 2
    grid_size = (grid_r, grid_c)
    table_position = [0.8, 0, Z_OFFSET]

    # ---------- Create a node ----------
    rospy.init_node("GazeboSceneGenerator")
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # ---------- Create services ----------
    objs = ObjectService(models_base_path=models_dir)
    scene_m = SceneMaker(MODEL_NAMES, models_dir, grid_size, table_position, TABLE_HEIGHT, stable_pose_file)

    # ---------- initialize clients for Fetch robot ----------
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()

    # ---------- initialize moveit components ----------
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander("arm")
    group.set_max_velocity_scaling_factor(1.0)
    group_grp = moveit_commander.MoveGroupCommander("gripper")
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )
    gripper = FetchGripper(group_grp)

    # Raise the torso
    rospy.loginfo("Raising torso...")
    torso_action.move_to([0.4])

    # look at table
    rospy.loginfo("Looking at the table...")
    if head_action.success:
        head_action.look_at(0.45, 0, TABLE_HEIGHT, "base_link")
    else:
        head_action = FollowTrajectoryClient("head_controller", ["head_pan_joint", "head_tilt_joint"])
        head_action.move_to([0.009195, 0.908270])

    # Create Scene in Gazebo
    sample_scene, object_names = create_scene()

    # Setup the planning scene
    do_motion_planning()

    # Clean up the scene in Gazebo
    cleanup_scene(object_names)
