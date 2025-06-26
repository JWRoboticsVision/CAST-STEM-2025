import os

os.environ["PYOPENGL_PLATFORM"] = "egl"
os.environ["MPLBACKEND"] = "Agg"  # Disable matplotlib GUI backend

import numpy as np
import torch
import trimesh

from fetch_grasp.utils import PROJECT_ROOT
from fetch_grasp.utils.commons import (
    read_rgb_image,
    read_depth_image,
    read_mask_image,
    write_rgb_image,
    write_data_to_yaml,
    write_mask_image,
    draw_annotated_image,
    extract_masks_from_labels,
    draw_image_overlay,
)
from fetch_grasp.rendering import OffscreenRenderer

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


if __name__ == "__main__":
    cam_RT = np.eye(4, dtype=np.float32)

    # Directories
    save_dir = PROJECT_ROOT / "demo/ros"
    models_dir = PROJECT_ROOT / "datasets/models"

    rgb_file = f"{save_dir}/color_image.png"
    depth_file = f"{save_dir}/depth_image.png"
    mask_file = f"{save_dir}/mask_image_035_power_drill.png"
    cam_K_file = f"{save_dir}/cam_K.txt"

    # Initialize NIDS-Net
    nids_model = initialize_nidsnet()

    # Initialize FoundationPose
    estimator = initialize_fdpose()

    # Load RGBD images and Camera K
    cam_K = np.loadtxt(cam_K_file, dtype=np.float32).reshape((3, 3))
    cam_RT = np.eye(4, dtype=np.float32)  # Identity matrix for camera pose
    mask = read_mask_image(mask_file)
    rgb = read_rgb_image(rgb_file)
    depth = read_depth_image(depth_file, scale=1000.0)
    depth[depth < 0.1] = 0
    depth[depth > 2.0] = 0

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
        print(f"{ob_in_cam}")

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
