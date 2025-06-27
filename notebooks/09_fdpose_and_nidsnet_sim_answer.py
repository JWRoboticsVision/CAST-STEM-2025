import os

os.environ["PYOPENGL_PLATFORM"] = "egl"

import cv2
import numpy as np
import torch
import trimesh

# ROS imports
import ros_numpy
import rospy
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from gazebo_msgs.srv import GetModelState
import tf2_ros
from tf.transformations import quaternion_matrix

from fetch_grasp.utils import PROJECT_ROOT
from fetch_grasp.utils.commons import (
    write_rgb_image,
    write_depth_image,
    write_mask_image,
    write_data_to_yaml,
    colorize_depth_image,
    draw_annotated_image,
    extract_masks_from_labels,
    draw_image_overlay,
)
from fetch_grasp.rendering import OffscreenRenderer
from fetch_grasp.utils.scene import ObjectService, SceneMaker
from fetch_grasp.utils.control import PointHeadClient, FollowTrajectoryClient
from fetch_grasp.utils.ros import ros_pose_to_rt

Z_OFFSET = -0.03  # difference between Real World and Gazebo table
TABLE_HEIGHT = 0.78 + Z_OFFSET

MODEL_NAMES = ["003_cracker_box", "035_power_drill"]

OBJECT_CLASSES = [
    "background",
    "002_master_chef_can",
    "003_cracker_box",
    "004_sugar_box",
    "005_tomato_soup_can",
    "006_mustard_bottle",
    "007_tuna_fish_can",
    "008_pudding_box",
    "009_gelatin_box",
    "010_potted_meat_can",
    "011_banana",
    "019_pitcher_base",
    "021_bleach_cleanser",
    "024_bowl",
    "025_mug",
    "035_power_drill",
    "036_wood_block",
    "037_scissors",
    "040_large_marker",
    "051_large_clamp",
    "052_extra_large_clamp",
    "061_foam_brick",
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


def get_camera_K(caminfo_topic):
    camInfo_msg = rospy.wait_for_message(caminfo_topic, CameraInfo)
    cam_model = PinholeCameraModel()
    cam_model.fromCameraInfo(camInfo_msg)
    K = cam_model.intrinsicMatrix().astype("float32")
    img_height, img_width = cam_model.height, cam_model.width

    return K, img_height, img_width


def get_cv_image(image_topic, message_type=Image):
    image_msg = rospy.wait_for_message(image_topic, message_type)
    img = ros_numpy.numpify(image_msg)
    if image_msg.encoding == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif image_msg.encoding == "bgr8":
        pass
    elif image_msg.encoding == "32FC1":
        pass
    elif image_msg.encoding == "16UC1":
        img = img.astype(np.float32) / 1000.0  # Convert to meters if depth
    else:
        raise ValueError(f"Unsupported image encoding: {image_msg.encoding}")
    return img


def run_nidsnet_once(nids_mod, image_rgb):
    _, labels = nids_mod.step(image_rgb)
    labels = labels.cpu().numpy().astype(np.uint8)
    masks = extract_masks_from_labels(labels)
    obj_names = [OBJECT_CLASSES[int(i)] for i in np.unique(labels) if i != 0]
    labels_vis = draw_annotated_image(image_rgb, masks=masks, labels=obj_names)
    return masks, obj_names, labels, labels_vis


def run_fdpose_register_once(est, mesh_file, rgb, depth, mask, K):
    # Reset object mesh
    mesh = trimesh.load(mesh_file, process=False)
    est.reset_object(model_pts=mesh.vertices.copy(), model_normals=mesh.vertex_normals.copy(), mesh=mesh)
    ob_in_cam_mat = est.register(rgb=rgb, depth=depth, ob_mask=mask, K=K, iteration=15)
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


def create_scene():
    rospy.loginfo("=" * 5 + " Creating scene in Gazebo... " + "=" * 5)
    # ---------- Add objects to the scene ----------
    sample_scene, _ = scene_m.create_scene()
    object_names = sorted(sample_scene.keys())

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


if __name__ == "__main__":
    base_frame = "base_link"
    camera_frame = "head_camera_rgb_optical_frame"

    # Define topics
    color_topic = "/head_camera/rgb/image_raw"
    depth_opic = "/head_camera/depth_registered/image_raw"
    caminfo_topic = "/head_camera/rgb/camera_info"

    # Directories
    save_dir = PROJECT_ROOT / "datasets/tmp/fdpose_nidsnet_sim"
    save_dir.mkdir(parents=True, exist_ok=True)
    models_dir = PROJECT_ROOT / "datasets/models"
    stable_pose_file = PROJECT_ROOT / "datasets/pose_data/selected_poses.pk"
    grid_size = (3, 2)
    table_position = [0.8, 0, Z_OFFSET]

    # Initialize ROS node
    rospy.init_node("fdpose_node", anonymous=True)
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # ---------- Create services ----------
    objs = ObjectService(models_base_path=models_dir)
    scene_m = SceneMaker(MODEL_NAMES, models_dir, grid_size, table_position, TABLE_HEIGHT, stable_pose_file)

    # ---------- initialize clients for Fetch robot ----------
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    torso_action.move_to([0.4])
    head_action.look_at(0.45, 0, TABLE_HEIGHT, "base_link")

    # Create Scene in Gazebo
    sample_scene, object_names = create_scene()

    # Get camera pose in base frame
    cam_in_base = get_tf_pose(tf_buffer, camera_frame, base_frame)
    print(f"Camera pose in base:\n{cam_in_base}")

    # Prepare input data
    bgr = get_cv_image(color_topic, Image)
    depth = get_cv_image(depth_opic, Image)
    depth_vis = colorize_depth_image(depth)
    cam_K, im_height, im_width = get_camera_K(caminfo_topic)
    cam_RT = np.eye(4, dtype=np.float32)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    # Save data
    write_rgb_image(f"{save_dir}/color_image.png", rgb)
    write_depth_image(f"{save_dir}/depth_image.png", (depth * 1000.0).astype(np.uint16))
    write_rgb_image(f"{save_dir}/depth_image_vis.png", depth_vis)
    np.savetxt(f"{save_dir}/cam_K.txt", cam_K, fmt="%.6f")

    # Initialize NIDS-Net
    nids_model = initialize_nidsnet()

    # Initialize FoundationPose
    estimator = initialize_fdpose()

    # Run NIDS-Net Inference
    print(f"Running NIDS-Net for object segmentation...")
    masks, obj_names, labels, labels_vis = run_nidsnet_once(nids_model, rgb)
    write_mask_image(f"{save_dir}/mask_image_nidsnet.png", labels)
    write_rgb_image(f"{save_dir}/mask_image_nidsnet_vis.png", labels_vis)
    write_data_to_yaml(
        f"{save_dir}/nidsnet_class_names.yaml", {int(i): OBJECT_CLASSES[int(i)] for i in np.unique(labels) if i != 0}
    )
    print(f"  - Detected objects: {obj_names}")

    # Run FoundationPose
    renderer = OffscreenRenderer(znear=0.1, zfar=100.0)
    renderer.add_camera(cam_K, "camera")

    print(f"Running FoundationPose for labeled objects...")
    ob_in_cam_poses = []
    ob_in_cam_poses_gt = []
    for obj_name, mask in zip(obj_names, masks):
        print(f"  - object: {obj_name}")
        mesh_file = f"{models_dir}/{obj_name}/textured_simple.obj"
        renderer.add_mesh(trimesh.load(mesh_file), obj_name)
        ob_in_cam = run_fdpose_register_once(
            est=estimator,
            mesh_file=mesh_file,
            rgb=rgb,
            depth=depth,
            mask=mask,
            K=cam_K,
        )
        ob_in_cam_poses.append(ob_in_cam)
        print(f"ob_in_cam:\n{ob_in_cam}")

        ob_in_base = np.matmul(cam_in_base, ob_in_cam)  # convert to base frame
        ob_in_base_gt = get_pose_gazebo(obj_name)  # get gt pose from Gazebo
        ob_in_cam_gt = np.matmul(np.linalg.inv(cam_in_base), ob_in_base_gt)  # convert to camera frame
        ob_in_cam_poses_gt.append(ob_in_cam_gt)

    # Save Object Poses
    np.savez_compressed(
        f"{save_dir}/ob_in_cam_poses.npz", **{name: pose for name, pose in zip(obj_names, ob_in_cam_poses)}
    )

    # Render FoundationPose results
    r_colors = renderer.get_render_colors(
        width=rgb.shape[1],
        height=rgb.shape[0],
        cam_names=["camera"],
        cam_poses=[cam_RT],
        mesh_names=obj_names,
        mesh_poses=ob_in_cam_poses,
    )
    vis = draw_image_overlay(rgb, r_colors[0], 0.75)
    write_rgb_image(f"{save_dir}/ob_in_cam_poses_vis.png", vis)

    # Render Groundtruth Object Poses
    r_colors = renderer.get_render_colors(
        width=rgb.shape[1],
        height=rgb.shape[0],
        cam_names=["camera"],
        cam_poses=[cam_RT],
        mesh_names=obj_names,
        mesh_poses=ob_in_cam_poses_gt,
    )
    vis = draw_image_overlay(rgb, r_colors[0], 0.75)
    write_rgb_image(f"{save_dir}/ob_in_cam_poses_gt_vis.png", vis)

    # Cleanup scene
    cleanup_scene(object_names)
