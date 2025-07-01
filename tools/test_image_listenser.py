import numpy as np
import torch
import trimesh

import rospy
import ros_numpy
from sensor_msgs.msg import Image

from fetch_grasp.utils import PROJECT_ROOT, OBJECT_CLASSES
from fetch_grasp.utils.commons import extract_masks_from_labels, draw_annotated_image, draw_image_overlay
from fetch_grasp.utils.image_listener import FetchImageListener
from fetch_grasp.rendering import OffscreenRenderer

MODEL_NAMES = [
    "003_cracker_box",
    "005_tomato_soup_can",
    "006_mustard_bottle",
    "007_tuna_fish_can",
    "008_pudding_box",
    "009_gelatin_box",
    "010_potted_meat_can",
    "011_banana",
    "021_bleach_cleanser",
    "024_bowl",
    "025_mug",
    "035_power_drill",
    "037_scissors",
    "040_large_marker",
    "052_extra_large_clamp",
]


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


def run_fdpose_register_once(est, obj_mesh, rgb, depth, mask, K):
    # Reset object mesh
    est.reset_object(model_pts=obj_mesh.vertices.copy(), model_normals=obj_mesh.vertex_normals.copy(), mesh=obj_mesh)
    # Run pose estimation with register mode
    ob_in_cam_mat = est.register(rgb=rgb.copy(), depth=depth.copy(), ob_mask=mask, K=K, iteration=15)
    return ob_in_cam_mat


def initialize_nidsnet():
    from fetch_grasp.wrappers.nidsnet import NIDS, feat_dict, weight_adapter_path

    print("Initializing NIDS-Net...")
    object_features = torch.Tensor(feat_dict["features"]).view(-1, 42, 1024).cuda()
    model = NIDS(template_features=object_features, use_adapter=True, adapter_path=weight_adapter_path)
    return model


def run_nidsnet_once(nids_mod, rgb):
    image_rgb = rgb.copy()
    _, labels = nids_mod.step(image_rgb)
    labels = labels.cpu().numpy().astype(np.uint8)
    masks = extract_masks_from_labels(labels)
    obj_names = [OBJECT_CLASSES[int(i)] for i in np.unique(labels) if i != 0]
    labels_vis = draw_annotated_image(image_rgb, masks=masks, labels=obj_names)
    return masks, obj_names, labels, labels_vis


def np_to_image_msg(np_image, frame_id, frame_stamp, encoding="rgb8"):
    """
    Convert a numpy image to a ROS Image message.
    """
    msg = ros_numpy.msgify(Image, np_image, encoding=encoding)
    msg.header.frame_id = frame_id
    msg.header.stamp = frame_stamp
    return msg


if __name__ == "__main__":
    models_dir = PROJECT_ROOT / "datasets/models"
    object_meshes = {obj_name: trimesh.load(models_dir / f"{obj_name}/textured_simple.obj") for obj_name in MODEL_NAMES}

    # Initialize NIDS-Net
    nids_mod = initialize_nidsnet()

    # Initialize FoundationPose
    fd_estimator = initialize_fdpose()

    # Initialize ROS node
    rospy.init_node("image_listener_test", anonymous=True)

    # Initialize image listener
    image_listener = FetchImageListener()
    cam_K = image_listener.cam_K
    im_width, im_height = image_listener.im_width, image_listener.im_height

    seg_vis_pub = rospy.Publisher("/fdpose/seg_vis", Image, queue_size=1)
    pose_vis_pub = rospy.Publisher("/fdpose/pose_vis", Image, queue_size=1)

    # Initialize OffscreenRenderer
    renderer = OffscreenRenderer()
    renderer.add_camera(cam_K, "camera")
    for obj_name, obj_mesh in object_meshes.items():
        renderer.add_mesh(obj_mesh, obj_name)

    while not rospy.is_shutdown():
        # Get RGB and depth images
        rgb, depth, cam_RT, cam_K = image_listener.get_data()
        frame_id = image_listener.rgb_frame_id
        frame_stamp = image_listener.rgb_frame_stamp
        if rgb is None:
            rospy.logwarn("Waiting for RGB and depth images...")
            continue

        masks, obj_names, labels, labels_vis = run_nidsnet_once(nids_mod, rgb)
        valid_obj_names = [name for name in obj_names if name in object_meshes]
        ob_in_cam_poses = [
            run_fdpose_register_once(
                fd_estimator, object_meshes[obj_name], rgb, depth, masks[obj_names.index(obj_name)], cam_K
            )
            for obj_name in valid_obj_names
        ]
        r_colors = renderer.get_render_colors(
            width=im_width,
            height=im_height,
            cam_names=["camera"],
            cam_poses=[np.eye(4)],
            mesh_names=valid_obj_names,
            mesh_poses=ob_in_cam_poses,
        )
        pose_vis = draw_image_overlay(rgb, r_colors[0], 0.7)

        seg_vis_pub.publish(np_to_image_msg(labels_vis, frame_id, frame_stamp, encoding="rgb8"))
        pose_vis_pub.publish(np_to_image_msg(pose_vis, frame_id, frame_stamp, encoding="rgb8"))
