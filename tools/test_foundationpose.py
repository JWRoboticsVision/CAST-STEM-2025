import os

os.environ["PYOPENGL_PLATFORM"] = "egl"
os.environ["MPLBACKEND"] = "Agg"  # Disable matplotlib GUI backend

import numpy as np
import matplotlib.pyplot as plt
import trimesh

from fetch_grasp.utils import PROJECT_ROOT
from fetch_grasp.utils.commons import (
    read_rgb_image,
    read_depth_image,
    read_mask_image,
    write_rgb_image,
    draw_image_overlay,
)
from fetch_grasp.rendering import OffscreenRenderer
from fetch_grasp.wrappers.foundationpose import FoundationPose, ScorePredictor, PoseRefinePredictor, set_seed, dr

# Load Input Data for FoundationPose
save_folder = PROJECT_ROOT / "demo/ros"
rgb_file = f"{save_folder}/color_image.png"
depth_file = f"{save_folder}/depth_image.png"
mask_file = f"{save_folder}/mask_image_035_power_drill.png"
cam_K_file = f"{save_folder}/cam_K.txt"
mesh_file = f"{PROJECT_ROOT}/third-party/SceneReplica/Datasets/benchmarking/models/035_power_drill/textured_simple.obj"

# camera intrinsic and extrinsic parameters
cam_K = np.loadtxt(cam_K_file, dtype=np.float32).reshape((3, 3))
cam_RT = np.eye(4, dtype=np.float32)  # Identity matrix for camera pose

# color, depth, and mask images
color = read_rgb_image(rgb_file)
depth = read_depth_image(depth_file, scale=1000.0)
depth[depth < 0.1] = 0
depth[depth > 2.0] = 0
mask = read_mask_image(mask_file)
im_height = color.shape[0]
im_width = color.shape[1]

# Object mesh
object_mesh = trimesh.load(mesh_file, process=False)

# Set random seed for reproducibility
set_seed(0)

estimator = FoundationPose(
    model_pts=object_mesh.vertices.astype(np.float32),
    model_normals=object_mesh.vertex_normals.astype(np.float32),
    mesh=object_mesh,
    scorer=ScorePredictor(),
    refiner=PoseRefinePredictor(),
    glctx=dr.RasterizeCudaContext(),
    debug=0,
    debug_dir="debug",
    rotation_grid_min_n_views=120,
    rotation_grid_inplane_step=60,
)

ob_in_cam_mat = estimator.register(
    rgb=color,
    depth=depth,
    ob_mask=mask,
    K=cam_K,
    iteration=15,
)
np.savetxt(save_folder / "ob_in_cam.txt", ob_in_cam_mat, fmt="%.6f")

print("Object in camera frame:")
print(ob_in_cam_mat)

# Initialize the renderer
renderer = OffscreenRenderer(znear=0.1, zfar=100.0)

# Add camera and object mesh to the renderer
renderer.add_camera(cam_K, "camera")
renderer.add_mesh(object_mesh, "object")

# Render color images with the given camera pose and object pose
r_colors = renderer.get_render_colors(
    width=im_width,
    height=im_height,
    cam_names=["camera"],
    cam_poses=[cam_RT],
    mesh_names=["object"],
    mesh_poses=[ob_in_cam_mat],
)

# Draw overlay on the original color image
vis = draw_image_overlay(color, r_colors[0], 0.75)

# Save the visual image
write_rgb_image(save_folder / "ob_in_cam_vis.png", vis)
