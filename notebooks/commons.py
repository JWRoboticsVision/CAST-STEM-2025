import os

os.environ["OPEN3D_DISABLE_WEB_VISUALIZER"] = "true"

from pathlib import Path
import json
import cv2
import numpy as np
import open3d as o3d

from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg

CURR_DIR = Path(__file__).resolve().parent
PROJ_ROOT = CURR_DIR.parent


def read_rgb_image(image_path):
    return cv2.cvtColor(cv2.imread(str(image_path)), cv2.COLOR_BGR2RGB)


def write_rgb_image(image_path, image):
    cv2.imwrite(str(image_path), image[:, :, ::-1])


def write_depth_image(image_path, image):
    cv2.imwrite(str(image_path), image)


def read_depth_image(image_path):
    return cv2.imread(str(image_path), cv2.IMREAD_ANYDEPTH)


def read_data_from_json(json_file):
    with open(str(json_file), "r", encoding="utf-8") as f:
        data = json.load(f)
    return data


def read_K_matrix_from_json(json_file, camera_type="color"):
    data = read_data_from_json(json_file)
    K = np.array(
        [
            [data[camera_type]["fx"], 0, data[camera_type]["ppx"]],
            [0, data[camera_type]["fy"], data[camera_type]["ppy"]],
            [0, 0, 1],
        ],
        dtype=np.float32,
    )
    return K


def read_extrinsics_from_json(json_file):
    data = read_data_from_json(json_file)
    cam_master = data["rs_master"]
    extrinsics = {}
    for key in data["extrinsics"]:
        extrinsics[key] = np.array(
            [
                [
                    data["extrinsics"][key][0],
                    data["extrinsics"][key][1],
                    data["extrinsics"][key][2],
                    data["extrinsics"][key][3],
                ],
                [
                    data["extrinsics"][key][4],
                    data["extrinsics"][key][5],
                    data["extrinsics"][key][6],
                    data["extrinsics"][key][7],
                ],
                [
                    data["extrinsics"][key][8],
                    data["extrinsics"][key][9],
                    data["extrinsics"][key][10],
                    data["extrinsics"][key][11],
                ],
                [0, 0, 0, 1],
            ],
            dtype=np.float32,
        )
    return extrinsics, cam_master


def get_xyz_from_uvd(uvd, fx, fy, cx, cy):
    """Convert 2D image point to 3D point

    Args:
        uvd: 2D point, shape (3,)
        fx: focal length in x
        fy: focal length in y
        cx: principal point in x
        cy: principal point in y
    Returns:
        xyz: 3D point, shape (3,)
    """
    u, v, d = uvd
    x = (u - cx) * d / fx
    y = (v - cy) * d / fy
    z = d
    return np.array([x, y, z], dtype=np.float32)


def get_uv_from_xyz(xyz, fx, fy, cx, cy):
    """Project 3D point to 2D image plane

    Args:
        xyz: 3D point, shape (3,)
        fx: focal length in x
        fy: focal length in y
        cx: principal point in x
        cy: principal point in y
    Returns:
        uv: 2D point, shape (2,)
    """
    x, y, z = xyz
    u = x * fx / z + cx
    v = y * fy / z + cy
    return np.array([u, v], dtype=np.int64)


def draw_landmarks_on_image(image, landmarks, color=(0, 255, 0), radius=3):
    vis = image.copy()
    for landmark in landmarks:
        x, y = landmark
        cv2.circle(vis, (int(x), int(y)), radius, color, -1)
    return vis


def deproject_depth_image(depth_img, cam_K, cam_RT=None):
    """
    Deproject depth image into 3D point cloud

    Args:
        depth_img: depth image, shape (H, W)
        cam_K: camera intrinsic parameters, shape (3, 3)
        cam_RT: camera extrinsic parameters, shape (4, 4)
    Returns:
        points_3d: 3D points, shape (N, 3)
    """
    H, W = depth_img.shape

    # Create a meshgrid of pixel coordinates
    u_coords, v_coords = np.meshgrid(np.arange(W), np.arange(H), indexing="xy")
    u_coords = u_coords.flatten()
    v_coords = v_coords.flatten()

    # Flatten the depth image to match the pixel coordinates
    z_coords = depth_img.flatten()

    # Convert pixel coordinates to normalized image coordinates
    x_coords = (u_coords - cam_K[0, 2]) * z_coords / cam_K[0, 0]
    y_coords = (v_coords - cam_K[1, 2]) * z_coords / cam_K[1, 1]

    # Stack the coordinates to form homogeneous coordinates
    points_3d = np.vstack((x_coords, y_coords, z_coords))  # Shape (3, N)
    points_3d = points_3d.T  # Shape (N, 3)

    # Apply the extrinsic transformation to obtain the 3D points in the world coordinate system
    if cam_RT is not None:
        points_3d = apply_transformation(points_3d, cam_RT)

    return points_3d


def apply_transformation(points, transformation):
    """
    Apply a 4x4 transformation matrix to a set of 3D points.

    Args:
    points (numpy.ndarray): Array of shape (N, 3) representing N points.
    transformation (numpy.ndarray): 4x4 transformation matrix.

    Returns:
    numpy.ndarray: Transformed points of shape (N, 3).
    """
    # Convert points from (N, 3) to (N, 4) by appending ones
    homogeneous_points = np.hstack([points, np.ones((len(points), 1))])

    # Apply the transformation matrix to the homogeneous points
    transformed_homogeneous_points = homogeneous_points.dot(transformation.T)

    # Convert back from homogeneous coordinates to (N, 3)
    transformed_points = transformed_homogeneous_points[:, :3]
    return transformed_points


def create_video_from_rgb_images(video_path, rgb_images, fps=30):
    if not rgb_images:
        raise ValueError("The list of RGB images is empty.")

    try:
        h, w, channels = rgb_images[0].shape
        if channels != 3:
            raise ValueError("Images must be 3-channel RGB.")
    except AttributeError:
        raise ValueError(
            "Invalid image data. Ensure all images are in the correct format and dimension."
        )

    if not all(img.shape == (h, w, 3) for img in rgb_images):
        raise ValueError(
            "All RGB images must have the same dimensions and number of channels."
        )

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video = cv2.VideoWriter(str(video_path), fourcc, fps, (w, h))
    try:
        for image in rgb_images:
            video.write(image[:, :, ::-1])  # Convert RGB to BGR
    finally:
        video.release()


def display_images(
    images,
    names=None,
    cmap="gray",
    figsize=(19.2, 10.8),
    dpi=100,
    max_cols=4,
    facecolor="white",
    save_path=None,
    return_array=False,
    idx=None,
):
    num_images = len(images)
    num_cols = min(num_images, max_cols)
    num_rows = (
        num_images + num_cols - 1
    ) // num_cols  # More efficient ceiling division
    if names is None:
        names = [f"fig_{i}" for i in range(num_images)]

    fig, axs = plt.subplots(
        num_rows, num_cols, figsize=figsize, dpi=dpi, facecolor=facecolor
    )

    # Flatten axs for easy iteration and handle cases with single subplot
    if num_images == 1:
        axs = [axs]
    else:
        axs = axs.flat

    try:
        for i, (image, name) in enumerate(zip(images, names)):
            ax = axs[i]
            if image.ndim == 3 and image.shape[2] == 3:  # RGB images
                ax.imshow(image)
            else:  # Depth or grayscale images
                ax.imshow(image, cmap=cmap)
            ax.set_title(name)
            ax.axis("off")

        # Hide any unused axes
        for j in range(i + 1, len(axs)):
            axs[j].axis("off")

        plt.tight_layout()

        img_array = None
        if return_array:
            canvas = FigureCanvasAgg(fig)
            canvas.draw()
            width, height = fig.get_size_inches() * fig.get_dpi()
            img_array = np.frombuffer(canvas.tostring_rgb(), dtype="uint8").reshape(
                int(height), int(width), 3
            )

        if save_path:
            plt.savefig(str(save_path))

        if not save_path and not return_array:
            plt.show()

    finally:
        plt.close(fig)

    if return_array:
        return img_array if idx is None else (img_array, idx)
