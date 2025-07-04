import copy
import sys
import numpy as np
import cv2
import torch
import trimesh

# ROS imports
import ros_numpy
import rospy
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.srv import GetModelState
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
import tf2_ros
from tf.transformations import quaternion_matrix

from fetch_grasp.grippers import FetchGripper
from fetch_grasp.utils import PROJECT_ROOT, OBJECT_CLASSES
from fetch_grasp.utils.commons import draw_image_overlay, extract_masks_from_labels, draw_annotated_image
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
from fetch_grasp.rendering import OffscreenRenderer

Z_OFFSET = -0.03  # difference between Real World and Gazebo table
TABLE_HEIGHT = 0.78 + Z_OFFSET

MODEL_NAMES = [
    "003_cracker_box",
    "004_sugar_box",
    "005_tomato_soup_can",
    "006_mustard_bottle",
    # "007_tuna_fish_can",
    "008_pudding_box",
    # "009_gelatin_box",
    # "010_potted_meat_can",
    "011_banana",
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
        transform = tf_buffer.lookup_transform(base_frame, target_frame, rospy.Time(0), rospy.Duration(1.0)).transform
        q = transform.rotation
        t = transform.translation
        RT_obj = quaternion_matrix([q.x, q.y, q.z, q.w])
        RT_obj[:3, 3] = [t.x, t.y, t.z]
        return RT_obj
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ):
        return None


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
            # scene.remove_world_object(obj_name)
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
    obj_mesh = object_meshes[object_name]
    while True:
        rgb = get_image_by_topic(color_topic, message_type=Image)
        depth = get_image_by_topic(depth_opic, message_type=Image)
        cam_RT = get_tf_pose(tf_buffer, camera_frame, base_frame)
        if rgb is not None and depth is not None and cam_RT is not None:
            break
    masks, obj_names, labels, labels_vis = run_nidsnet_once(nids_mod, rgb)
    nids_vis_pub.publish(ros_numpy.msgify(Image, labels_vis, encoding="rgb8"))
    if object_name not in obj_names:
        print(f"Object {object_name} not found in NIDS-Net output.")
        return None
    mask = masks[obj_names.index(object_name)]  # get the mask for the object
    ob_in_cam = run_fdpose_register_once(estimator, obj_mesh, rgb, depth, mask, cam_K)
    obj_RT = np.matmul(cam_RT, ob_in_cam)  # object pose in base frame

    r_colors = renderer.get_render_colors(
        width=rgb.shape[1],
        height=rgb.shape[0],
        cam_names=["camera"],
        cam_poses=[np.eye(4)],
        mesh_names=[object_name],
        mesh_poses=[ob_in_cam],
    )
    poses_vis = draw_image_overlay(rgb, r_colors[0], 0.65)
    pose_vis_pub.publish(ros_numpy.msgify(Image, poses_vis, encoding="rgb8"))

    return obj_RT


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
    display_trajectory = DisplayTrajectory()
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
    display_trajectory = DisplayTrajectory()
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
    RT_gripper = get_tf_pose(tf_buffer, "ati_link", "base_link")
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
        scene.add_mesh(obj_name, p, f"{models_dir}/{obj_name}/textured_simple.obj")


def do_motion_planning():
    # ---------- Find observed objects on the table ----------
    while True:
        rgb = get_image_by_topic(color_topic, message_type=Image)
        if rgb is not None:
            break
    masks, obj_names, labels, labels_vis = run_nidsnet_once(nids_mod, rgb)
    nids_vis_pub.publish(ros_numpy.msgify(Image, labels_vis, encoding="rgb8"))

    object_names = [name for name in obj_names if name in MODEL_NAMES]

    if len(object_names) == 0:
        print(f"No Valid Objects found on Table!!!")
        return

    print(f"Observed Objects on Table: {object_names}")

    # ---------- Setup the planning scene ----------
    target_object_names = [n for n in object_names]
    for obj_idx, obj_name in enumerate(object_names):
        print("=" * 5 + f" Start Motion Planning for {obj_name} " + "=" * 5)
        grasp_num, trajectory_standoff, trajectory_final = None, None, None
        gripper.open()
        rospy.loginfo("** Setting up the planning scene...")
        setup_planning_scene(scene, target_object_names, pose_method)

        # get the object pose in robot base frame
        RT_obj = get_pose(obj_name, pose_method)

        direct_topdown = False
        if not direct_topdown:
            # Current gripper pose
            RT_gripper = get_gripper_rt(tf_buffer)
            print("RT_gripper", RT_gripper)
            # Using Graspit generated grasp to test the Pose Detection (model based grasping)

            # Load grasps
            grasp_file = f"{grasp_dir}/fetch_gripper-{obj_name}.json"
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
            mesh_p = f"{models_dir}/{obj_name}/textured_simple.obj"
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
        else:
            print("Trying to lift object")
            RT_gripper = get_gripper_rt(tf_buffer)
            print("RT_gripper before lifting\n", RT_gripper)
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
                print("RT_gripper after lift\n", RT_gripper)
                move_arm_to_dropoff(group, RT_gripper, x_final=0.78)
                if gripper.is_fully_closed() or gripper.is_fully_open():
                    print("Gripper fully open/closed (after Moving)....")
                print(f"{obj_name} successfully droppedoff")

        # ------------------------ OPEN GRIPPER & STOW ---------------------#
        input("Open Gripper??")
        gripper.open()
        print("STOWING THE GRIPPER")
        reset_arm_stow(group)

        target_object_names.remove(obj_name)  # remove the object from the target list

        for i in range(3):
            x = input("proceed next object?").lower()
            if x == "n":
                break

    # Clear Planning Scene
    scene.clear()
    rospy.sleep(1.0)
    scene.remove_world_object()


def get_camera_K(caminfo_topic):
    camInfo_msg = rospy.wait_for_message(caminfo_topic, CameraInfo)
    cam_model = PinholeCameraModel()
    cam_model.fromCameraInfo(camInfo_msg)
    K = cam_model.intrinsicMatrix().astype("float32")
    img_height, img_width = cam_model.height, cam_model.width

    return K, img_height, img_width


def get_image_by_topic(image_topic, message_type=Image):
    image_msg = rospy.wait_for_message(image_topic, message_type)
    img = ros_numpy.numpify(image_msg)
    if image_msg.encoding == "rgb8":
        pass
    elif image_msg.encoding == "bgr8":
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    elif image_msg.encoding == "32FC1":
        pass
    elif image_msg.encoding == "16UC1":
        img = img.astype(np.float32) / 1000.0  # Convert to meters if depth
    else:
        raise ValueError(f"Unsupported image encoding: {image_msg.encoding}")
    return img


def run_nidsnet_once(nids_mod, image_rgb):
    _, labels = nids_mod.step(image_rgb.copy())
    labels = labels.cpu().numpy().astype(np.uint8)
    masks = extract_masks_from_labels(labels)
    obj_names = [OBJECT_CLASSES[int(i)] for i in np.unique(labels) if i != 0]
    labels_vis = draw_annotated_image(image_rgb, masks=masks, labels=obj_names)
    return masks, obj_names, labels, labels_vis


def run_fdpose_register_once(est, obj_mesh, rgb, depth, mask, K):
    # Reset object mesh
    est.reset_object(model_pts=obj_mesh.vertices.copy(), model_normals=obj_mesh.vertex_normals.copy(), mesh=obj_mesh)
    # Run pose estimation with register mode
    ob_in_cam_mat = est.register(rgb=rgb.copy(), depth=depth.copy(), ob_mask=mask, K=K, iteration=15)
    return ob_in_cam_mat


def initialize_fdpose():
    from fetch_grasp.wrappers.foundationpose import FoundationPose, ScorePredictor, PoseRefinePredictor, set_seed, dr

    set_seed(0)

    print("Initializing FoundationPose...")

    # Create a dummy mesh from box primitive mesh
    m_box = trimesh.primitives.Box()
    dummy_mesh = trimesh.Trimesh(vertices=m_box.vertices, faces=m_box.faces, vertex_normals=m_box.vertex_normals)

    estimator = FoundationPose(
        model_pts=dummy_mesh.vertices.astype(np.float32),
        model_normals=dummy_mesh.vertex_normals.astype(np.float32),
        mesh=dummy_mesh,
        scorer=ScorePredictor(),
        refiner=PoseRefinePredictor(),
        glctx=dr.RasterizeCudaContext(),
        rotation_grid_min_n_views=120,
        rotation_grid_inplane_step=60,
    )

    return estimator


def initialize_nidsnet():
    from fetch_grasp.wrappers.nidsnet import NIDS, feat_dict, weight_adapter_path

    print("Initializing NIDS-Net...")
    object_features = torch.Tensor(feat_dict["features"]).view(-1, 42, 1024).cuda()
    model = NIDS(template_features=object_features, use_adapter=True, adapter_path=weight_adapter_path)

    return model


if __name__ == "__main__":
    pose_method = "fdpose"  # gazebo, fdpose
    models_dir = PROJECT_ROOT / "datasets/models"
    grasp_dir = PROJECT_ROOT / "datasets/grasp_data/refined_grasps"
    stable_pose_file = PROJECT_ROOT / "datasets/pose_data/selected_poses.pk"
    table_position = [0.8, 0, Z_OFFSET]

    # Define topics
    color_topic = "/head_camera/rgb/image_raw"
    depth_opic = "/head_camera/depth_registered/image_raw"
    caminfo_topic = "/head_camera/rgb/camera_info"

    # Define frames for tf pose queries
    base_frame = "base_link"
    camera_frame = "head_camera_rgb_optical_frame"

    # ---------- Create a node ----------
    rospy.init_node("GazeboSceneGenerator")
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    nids_vis_pub = rospy.Publisher("/fdpose/nids_vis", Image, queue_size=1)
    pose_vis_pub = rospy.Publisher("/fdpose/pose_vis", Image, queue_size=1)
    cam_K, im_height, im_width = get_camera_K(caminfo_topic)

    # ---------- initialize clients for Fetch robot ----------
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()

    # ---------- initialize moveit components ----------
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander("arm")
    group.set_max_velocity_scaling_factor(0.5)
    group.set_end_effector_link("ati_link")
    group_grp = moveit_commander.MoveGroupCommander("gripper")
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)
    gripper = FetchGripper(group_grp)

    # ---------- initialize FoundationPose and NIDS-Net ----------
    if pose_method == "fdpose":
        rospy.loginfo("Initializing FoundationPose and NIDS-Net...")
        nids_mod = initialize_nidsnet()
        estimator = initialize_fdpose()
        # Preload object meshes
        object_meshes = {
            model_name: trimesh.load_mesh(f"{models_dir}/{model_name}/textured_simple.obj")
            for model_name in MODEL_NAMES
        }
        renderer = OffscreenRenderer(znear=0.1, zfar=100.0)
        renderer.add_camera(cam_K, "camera")
        for obj_name, obj_mesh in object_meshes.items():
            renderer.add_mesh(obj_mesh, obj_name)

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

    object_names = []

    while not rospy.is_shutdown():
        try:
            for i in range(3):
                x = input("Ready for next Scene?").lower()
                if x == "n":
                    y = input("Exit the program?").lower()
                    if y == "y":
                        print("Exiting the program...")
                        moveit_commander.roscpp_shutdown()
                        rospy.signal_shutdown("User requested shutdown.")
                        sys.exit(0)
                    break

            # Setup the planning scene
            do_motion_planning()

        except rospy.ROSInterruptException:
            print("ROS Interrupt Exception caught. Exiting...")
            break

        except Exception as e:
            print(f"An error occurred: {e}")
            break
