import logging
import os
from pathlib import Path
import sys
import shutil
import json
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import matplotlib.pyplot as plt
from matplotlib import colormaps
from matplotlib.backends.backend_agg import FigureCanvasAgg
import supervision as sv
import av


def compute_xyz(depth_img, fx, fy, px, py, height, width):
    indices = np.indices((height, width), dtype=np.float32).transpose(1, 2, 0)
    z_e = depth_img
    x_e = (indices[..., 1] - px) * z_e / fx
    y_e = (indices[..., 0] - py) * z_e / fy
    xyz_img = np.stack([x_e, y_e, z_e], axis=-1)  # Shape: [H x W x 3]
    return xyz_img


def deproject_depth_image(depth_img, cam_K):
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

    return points_3d


def colorize_depth_image(depth_image, colormap_name="jet"):
    """Colorizes a depth image using a matplotlib colormap."""
    valid_mask = (depth_image > 0.1) & (depth_image < 1.0)
    if not np.any(valid_mask):
        return np.zeros((*depth_image.shape, 3), dtype=np.uint8)
    # Normalize valid depth to [0, 1]
    depth_min = np.nanmin(depth_image[valid_mask])
    depth_max = np.nanmax(depth_image[valid_mask])
    depth_norm = np.clip((depth_image - depth_min) / (depth_max - depth_min), 0, 1)
    # Apply colormap
    colormap = colormaps.get_cmap(colormap_name)
    colored_image = colormap(depth_norm)[:, :, :3]  # drop alpha
    # Convert to uint8 RGB image
    rgb_image = (colored_image * 255).astype(np.uint8)
    return rgb_image


def _apply_morphology(mask, operation, kernel_size=3, iterations=1):
    """Helper function to apply a morphological operation (erode/dilate) on the mask."""
    if mask.ndim not in [2, 3]:
        raise ValueError("Mask must be a 2D or 3D numpy array.")
    if kernel_size <= 1:
        raise ValueError("Kernel size must be greater than 1.")
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask_dtype = mask.dtype
    mask = mask.astype(np.uint8)
    if operation == "erode":
        result = cv2.erode(mask, kernel, iterations=iterations)
    elif operation == "dilate":
        result = cv2.dilate(mask, kernel, iterations=iterations)
    else:
        raise ValueError(f"Invalid operation: {operation}. Use 'erode' or 'dilate'.")
    return result.astype(mask_dtype)


def _plot_image(ax, image, name, facecolor, titlecolor, fontsize):
    """Helper function to plot an image in the grid."""
    if image.ndim == 3 and image.shape[2] == 3:  # RGB image
        ax.imshow(image)
    elif image.ndim == 2 and image.dtype == np.uint8:  # Grayscale/mask image
        unique_values = np.unique(image)
        cmap = "tab10" if len(unique_values) <= 10 else "gray"
        ax.imshow(image, cmap=cmap)
    elif image.ndim == 2 and image.dtype == bool:  # Binary image
        ax.imshow(image, cmap="gray")
    else:  # Depth or other image
        ax.imshow(image, cmap="viridis")

    if name:
        ax.text(
            5,
            5,
            name,
            fontsize=fontsize,
            color=titlecolor,
            verticalalignment="top",
            horizontalalignment="left",
            bbox=dict(facecolor=facecolor, alpha=0.5, edgecolor="none", pad=3),
        )


def add_path(path):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


def get_logger(log_name="PoseToolkit", log_level="INFO", log_file=None):
    """Create and return a logger with console and optional file output."""
    logger = logging.getLogger(log_name)
    logger.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        "[%(asctime)s] [%(name)s:%(funcName)s] [%(levelname).3s] %(message)s",
        datefmt="%Y%m%d;%H:%M:%S",
    )
    if not logger.hasHandlers():
        if log_file:
            fh = logging.FileHandler(log_file)
            fh.setLevel(logging.DEBUG)
            fh.setFormatter(formatter)
            logger.addHandler(fh)
        # Console handler
        ch = logging.StreamHandler()
        ch.setLevel(getattr(logging, log_level.upper(), logging.INFO))
        ch.setFormatter(formatter)
        logger.addHandler(ch)
    return logger


def make_clean_folder(folder_path):
    folder = Path(folder_path)
    if folder.exists():
        shutil.rmtree(folder)
    folder.mkdir(parents=True)


def copy_file(src_path, dst_path):
    """Copy a file from the source path to the destination path."""
    if not Path(src_path).is_file():
        raise FileNotFoundError(f"Source file does not exist: '{src_path}'.")
    Path(dst_path).parent.mkdir(parents=True, exist_ok=True)
    try:
        shutil.copyfile(str(src_path), str(dst_path))
    except OSError as e:
        raise OSError(f"Failed to copy file from '{src_path}' to '{dst_path}': {e}")


def read_data_from_json(json_path):
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data


def write_data_to_json(json_path, data):
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False, sort_keys=False)


def read_data_from_yaml(yaml_path):
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data


def write_data_to_yaml(yaml_path, data):
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.dump(
            data,
            f,
            default_flow_style=False,
            allow_unicode=False,
            sort_keys=False,
            indent=2,
        )


def apply_transformation(points, trans_mat):
    homo_points = np.stack([points, np.ones((points.shape[0], 1))])
    homo_points = homo_points.dot(trans_mat.T)
    return homo_points[:, :3]


def rvt_to_quat(rvt):
    """Convert rotation vector and translation vector to quaternion and translation vector.

    Args:
        rvt (np.ndarray): Rotation vector and translation vector, shape (6,) or (N, 6). [rvx, rvy, rvz, tx, ty, tz]

    Returns:
        np.ndarray: Quaternion and translation vector, shape (7,) or (N, 7), [qx, qy, qz, qw, tx, ty, tz].

    Raises:
        ValueError: If the input does not have the expected shape or dimensions.
    """
    if not isinstance(rvt, np.ndarray) or rvt.shape[-1] != 6:
        raise ValueError("Input must be a numpy array with last dimension size 6.")

    if rvt.ndim == 2:
        rv = rvt[:, :3]
        t = rvt[:, 3:]
    elif rvt.ndim == 1:
        rv = rvt[:3]
        t = rvt[3:]
    else:
        raise ValueError("Input must be either 1D or 2D with a last dimension size of 6.")

    r = R.from_rotvec(rv)
    q = r.as_quat()  # this will be (N, 4) if rv is (N, 3), otherwise (4,)
    if q.ndim == 1:
        return np.concatenate((q, t))  # 1D case
    return np.concatenate((q, t), axis=-1)  # 2D case


def rvt_to_mat(rvt):
    """Convert rotation vector and translation vector to pose matrix.

    Args:
        rvt (np.ndarray): Rotation vector and translation vector, shape (6,) or (N, 6). [rvx, rvy, rvz, tx, ty, tz]
    Returns:
        np.ndarray: Pose matrix, shape (4, 4) or (N, 4, 4).

    Raises:
        ValueError: If the input does not have the expected shape or dimensions.
    """
    if not isinstance(rvt, np.ndarray) or rvt.shape[-1] != 6:
        raise ValueError("Input must be a numpy array with last dimension size 6.")

    if rvt.ndim == 1:
        p = np.eye(4)
        rv = rvt[:3]
        t = rvt[3:]
        r = R.from_rotvec(rv)
        p[:3, :3] = r.as_matrix()
        p[:3, 3] = t
        return p.astype(np.float32)
    elif rvt.ndim == 2:
        p = np.eye(4).reshape((1, 4, 4)).repeat(len(rvt), axis=0)
        rv = rvt[:, :3]
        t = rvt[:, 3:]
        r = R.from_rotvec(rv)
        for i in range(len(rvt)):
            p[i, :3, :3] = r[i].as_matrix()
            p[i, :3, 3] = t[i]
        return p.astype(np.float32)
    else:
        raise ValueError("Input must be either 1D or 2D with a last dimension size of 6.")


def mat_to_rvt(mat_4x4):
    """Convert pose matrix to rotation vector and translation vector.

    Args:
        mat_4x4 (np.ndarray): Pose matrix, shape (4, 4) or (N, 4, 4).
    Returns:
        np.ndarray: Rotation vector and translation vector, shape (6,) or (N, 6).

    Raises:
        ValueError: If the input does not have the expected shape or dimensions.
    """
    if not isinstance(mat_4x4, np.ndarray) or mat_4x4.shape[-2:] != (4, 4):
        raise ValueError("Input must be a numpy array with shape (4, 4) or (N, 4, 4).")

    if mat_4x4.ndim == 2:  # Single matrix input
        r = R.from_matrix(mat_4x4[:3, :3])
        rv = r.as_rotvec()
        t = mat_4x4[:3, 3]
        return np.concatenate([rv, t], dtype=np.float32)
    elif mat_4x4.ndim == 3:  # Batch of matrices
        rv = np.empty((len(mat_4x4), 3), dtype=np.float32)
        t = mat_4x4[:, :3, 3]
        for i, mat in enumerate(mat_4x4):
            r = R.from_matrix(mat[:3, :3])
            rv[i] = r.as_rotvec()
        return np.concatenate([rv, t], axis=1, dtype=np.float32)
    else:
        raise ValueError("Input dimension is not valid. Must be 2D or 3D.")


def mat_to_quat(mat_4x4):
    """
    Convert pose matrix to quaternion and translation vector.

    Args:
        mat_4x4 (np.ndarray): Pose matrix, shape (4, 4) for single input or (N, 4, 4) for batched input.

    Returns:
        np.ndarray: Quaternion and translation vector, shape (7,) for single input or (N, 7) for batched input.

    Raises:
        ValueError: If the input does not have the expected shape or dimensions.
    """
    if not isinstance(mat_4x4, np.ndarray) or mat_4x4.shape[-2:] != (4, 4):
        raise ValueError("Input must be a numpy array with shape (4, 4) or (N, 4, 4).")

    if np.all(mat_4x4 == -1):
        if mat_4x4.ndim == 2:  # Single matrix (shape (4, 4))
            return np.full((7,), -1, dtype=np.float32)
        elif mat_4x4.ndim == 3:  # Batch of matrices (shape (N, 4, 4))
            return np.full((mat_4x4.shape[0], 7), -1, dtype=np.float32)

    if mat_4x4.ndim == 2:  # Single matrix (shape (4, 4))
        r = R.from_matrix(mat_4x4[:3, :3])
        q = r.as_quat()  # Quaternion (shape (4,))
        t = mat_4x4[:3, 3]  # Translation (shape (3,))
        return np.concatenate([q, t], dtype=np.float32)

    elif mat_4x4.ndim == 3:  # Batch of matrices (shape (N, 4, 4))
        r = R.from_matrix(mat_4x4[:, :3, :3])  # Handle batch of rotation matrices
        q = r.as_quat()  # Quaternions (shape (N, 4))
        t = mat_4x4[:, :3, 3]  # Translations (shape (N, 3))
        return np.concatenate([q, t], axis=-1).astype(np.float32)  # Shape (N, 7)

    else:
        raise ValueError("Input dimension is not valid. Must be 2D or 3D.")


def quat_to_mat(quat):
    """Convert quaternion and translation vector to pose matrix.

    Args:
        quat (np.ndarray): Quaternion and translation vector, shape (7,) or (N, 7).
    Returns:
        np.ndarray: Pose matrix, shape (4, 4) or (N, 4, 4).

    Raises:
        ValueError: If the input does not have the last dimension size of 7.
    """
    if quat.shape[-1] != 7:
        raise ValueError("Input must have the last dimension size of 7.")

    batch_mode = quat.ndim == 2
    q = quat[..., :4]
    t = quat[..., 4:]

    if batch_mode:
        p = np.eye(4).reshape(1, 4, 4).repeat(len(quat), axis=0)
    else:
        p = np.eye(4)

    r = R.from_quat(q)
    p[..., :3, :3] = r.as_matrix()
    p[..., :3, 3] = t

    return p.astype(np.float32)


def quat_to_rvt(quat):
    """Convert quaternion and translation vector to rotation vector and translation vector.

    Args:
        quat (np.ndarray): Quaternion and translation vector, shape (7,) or (N, 7).
    Returns:
        np.ndarray: Rotation vector and translation vector, shape (6,) or (N, 6).

    Raises:
        ValueError: If the input does not have the last dimension size of 7.
    """
    if quat.shape[-1] != 7:
        raise ValueError("Input must have the last dimension size of 7.")

    batch_mode = quat.ndim == 2
    q = quat[..., :4]
    t = quat[..., 4:]

    r = R.from_quat(q)
    rv = r.as_rotvec()

    if batch_mode:
        return np.concatenate(
            [rv, t], axis=-1, dtype=np.float32
        )  # Ensure that the right axis is used for batch processing
    else:
        return np.concatenate([rv, t], dtype=np.float32)  # No axis needed for 1D arrays


def read_rgb_image(file_path):
    """Read an RGB image from the specified file path."""
    image = cv2.imread(str(file_path))
    if image is None:
        raise ValueError(f"Failed to load image from '{file_path}'.")
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image


def write_rgb_image(file_path, image):
    """Write an RGB image to the specified file path."""
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Input image must be an RGB image with 3 channels.")
    success = cv2.imwrite(str(file_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    if not success:
        raise ValueError(f"Failed to write RGB image to '{file_path}'.")


def read_depth_image(file_path, scale=1000.0):
    """Read a depth image from the specified file path."""
    image = cv2.imread(str(file_path), cv2.IMREAD_ANYDEPTH)
    if image is None:
        raise ValueError(f"Failed to load depth image from '{file_path}'.")
    image = image.astype(np.float32) / scale
    return image


def write_depth_image(file_path, image):
    """Write a depth image to the specified file path."""
    if image.dtype not in [np.uint16, np.uint8, np.float32]:
        raise ValueError("Depth image must be of type uint16,uint8 or float32.")
    success = cv2.imwrite(str(file_path), image)
    if not success:
        raise ValueError(f"Failed to write depth image to '{file_path}'.")


def read_mask_image(file_path):
    """Read a mask image from the specified file path."""
    image = cv2.imread(str(file_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"Failed to load mask image from '{file_path}'.")
    return image


def write_mask_image(file_path, image):
    """Write a mask image to the specified file path."""
    success = cv2.imwrite(str(file_path), image)
    if not success:
        raise ValueError(f"Failed to write mask image to '{file_path}'.")


def create_video_from_rgb_images(file_path, rgb_images, fps: int = 30):
    """Create a video from a list of RGB images."""
    if not rgb_images:
        raise ValueError("The list of RGB images is empty.")
    height, width = rgb_images[0].shape[:2]
    container = None
    try:
        container = av.open(str(file_path), mode="w")
        stream = container.add_stream("h264", rate=fps)
        stream.width = width
        stream.height = height
        stream.pix_fmt = "yuv420p"
        stream.thread_type = "FRAME"  # Parallel processing of frames
        stream.thread_count = os.cpu_count() or 1  # Number of threads to use
        for image in rgb_images:
            frame = av.VideoFrame.from_ndarray(image, format="rgb24")
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
    except Exception as e:
        raise IOError(f"Failed to write video to '{file_path}': {e}")
    finally:
        if container:
            container.close()


def erode_mask(mask, kernel_size=3, interations=1):
    if kernel_size <= 1:
        return mask
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    m_dtype = mask.dtype
    mask = mask.astype(np.uint8)
    mask = cv2.erode(mask, kernel, iterations=interations)
    return mask.astype(m_dtype)


def dilate_mask(mask, kernel_size=3, interations=1):
    if kernel_size <= 1:
        return mask
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    m_dtype = mask.dtype
    mask = mask.astype(np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=interations)
    return mask.astype(m_dtype)


def adjust_xyxy_bbox(bbox, width, height, margin=3):
    """
    Adjust bounding box coordinates with margins and boundary conditions.

    Args:
        bbox (list of int or float): Bounding box coordinates [x_min, y_min, x_max, y_max].
        width (int): Width of the image or mask. Must be greater than 0.
        height (int): Height of the image or mask. Must be greater than 0.
        margin (int): Margin to be added to the bounding box. Must be non-negative.

    Returns:
        np.ndarray: Adjusted bounding box as a numpy array.

    Raises:
        ValueError: If inputs are not within the expected ranges or types.
    """
    if len(bbox) != 4:
        raise ValueError("Bounding box must contain exactly four coordinates.")
    if not all(isinstance(x, (int, float)) for x in bbox):
        raise ValueError("Bounding box coordinates must be integers or floats.")
    if not isinstance(width, int) or not isinstance(height, int) or not isinstance(margin, int):
        raise ValueError("Width, height, and margin must be integers.")
    if width <= 0 or height <= 0:
        raise ValueError("Width and height must be positive integers.")
    if margin < 0:
        raise ValueError("Margin must be a non-negative integer.")

    # Convert bbox to integers if necessary
    x_min, y_min, x_max, y_max = map(int, bbox)

    x_min = max(0, x_min - margin)
    y_min = max(0, y_min - margin)
    x_max = min(width - 1, x_max + margin)
    y_max = min(height - 1, y_max + margin)
    return np.array([x_min, y_min, x_max, y_max], dtype=np.int64)


def get_bbox_from_landmarks(landmarks, width, height, margin=3):
    """
    Calculate a bounding box from a set of landmarks, adding an optional margin.

    Args:
        landmarks (np.ndarray): Landmarks array, shape (num_points, 2), with points marked as [-1, -1] being invalid.
        width (int): Width of the image or frame from which landmarks were extracted.
        height (int): Height of the image or frame.
        margin (int): Margin to add around the calculated bounding box. Default is 3.

    Returns:
        np.ndarray: Bounding box coordinates as [x_min, y_min, x_max, y_max].

    Raises:
        ValueError: If landmarks are not a 2D numpy array with two columns, or if width, height, or margin are non-positive.
    """
    if not isinstance(landmarks, np.ndarray) or landmarks.ndim != 2 or landmarks.shape[1] != 2:
        raise ValueError("Landmarks must be a 2D numpy array with shape (num_points, 2).")
    if not all(isinstance(i, int) and i > 0 for i in [width, height, margin]):
        raise ValueError("Width, height, and margin must be positive integers.")

    valid_marks = landmarks[~np.any(landmarks == -1, axis=1)]
    if valid_marks.size == 0:
        raise ValueError("No valid landmarks found; all landmarks are marked as invalid.")

    x, y, w, h = cv2.boundingRect(valid_marks)
    bbox = np.array([x - margin, y - margin, x + w + margin, y + h + margin], dtype=np.int64)
    bbox[:2] = np.maximum(0, bbox[:2])  # Adjust x_min and y_min
    bbox[2] = min(width - 1, bbox[2])  # Adjust x_max
    bbox[3] = min(height - 1, bbox[3])  # Adjust y_max

    return bbox


def get_bbox_from_mask(mask, margin=3):
    """
    Calculate a bounding box from a binary mask with an optional margin.

    Args:
        mask (np.ndarray): Binary mask, shape (height, width), where non-zero values indicate areas of interest.
        margin (int): Margin to add around the calculated bounding box. Must be non-negative.

    Returns:
        np.ndarray: Adjusted bounding box coordinates as [x_min, y_min, x_max, y_max].

    Raises:
        ValueError: If the mask is not a 2D array or contains no non-zero values, or if the margin is negative.
    """
    if not isinstance(mask, np.ndarray) or mask.ndim != 2:
        raise ValueError("Mask must be a 2D numpy array.")
    if margin < 0:
        raise ValueError("Margin must be non-negative.")
    if not np.any(mask):
        raise ValueError("Mask contains no non-zero values; cannot determine bounding rectangle.")

    height, width = mask.shape
    mask_uint8 = mask.astype(np.uint8)  # Ensure mask is in appropriate format for cv2.boundingRect
    x, y, w, h = cv2.boundingRect(mask_uint8)
    bbox = [x, y, x + w, y + h]
    bbox[0] = max(0, bbox[0] - margin)
    bbox[1] = max(0, bbox[1] - margin)
    bbox[2] = min(width - 1, bbox[2] + margin)
    bbox[3] = min(height - 1, bbox[3] + margin)

    return np.array(bbox, dtype=np.int64)


def xyxy_to_cxcywh(bbox):
    """
    Convert bounding box coordinates from top-left and bottom-right (XYXY) format
    to center-x, center-y, width, and height (CXCYWH) format.

    Args:
        bbox (np.ndarray): Bounding box array in XYXY format. Should be of shape (4,) for a single box
                            or (N, 4) for multiple boxes, where N is the number of boxes.

    Returns:
        np.ndarray: Converted bounding box array in CXCYWH format, with the same shape as the input.

    Raises:
        ValueError: If the input is not 1D or 2D with the last dimension size of 4.
    """
    bbox = np.asarray(bbox)
    if bbox.ndim not in [1, 2] or bbox.shape[-1] != 4:
        raise ValueError("Input array must be 1D or 2D with the last dimension size of 4.")

    # Calculate the center coordinates, width, and height
    cx = (bbox[..., 0] + bbox[..., 2]) / 2
    cy = (bbox[..., 1] + bbox[..., 3]) / 2
    w = bbox[..., 2] - bbox[..., 0]
    h = bbox[..., 3] - bbox[..., 1]

    return np.stack([cx, cy, w, h], axis=-1)


def draw_image_grid(
    images,
    names=None,
    figsize=(1920, 1080),
    max_cols=4,
    facecolor="white",
    titlecolor="black",
    fontsize=12,
    bar_width=0.2,
):
    """Display a list of images in a grid and draw the title name on each image's top-left corner."""
    num_images = len(images)
    if num_images == 0:
        raise ValueError("No images provided to display.")
    num_cols = min(num_images, max_cols)
    num_rows = (num_images + num_cols - 1) // num_cols
    # Default to no names if not provided
    if names is None or len(names) != num_images:
        names = [None] * num_images
    # Create figure and axis grid
    fig, axs = plt.subplots(
        num_rows,
        num_cols,
        figsize=(figsize[0] / 100.0, figsize[1] / 100.0),
        dpi=100,
        facecolor=facecolor,
    )
    axs = np.atleast_1d(axs).flat  # Ensure axs is always iterable
    # Plot each image
    for i, (image, name) in enumerate(zip(images, names)):
        _plot_image(axs[i], image, name, facecolor, titlecolor, fontsize)
        axs[i].axis("off")
    # Hide unused axes
    for ax in axs[i + 1 :]:
        ax.axis("off")
    # Adjust layout and spacing
    plt.tight_layout(pad=bar_width, h_pad=bar_width, w_pad=bar_width)
    # Convert the figure to an RGB array
    fig.canvas.draw()
    rgb_image = np.array(fig.canvas.buffer_rgba())[:, :, :3]
    # Close the figure
    plt.close(fig)
    return rgb_image


def draw_image_overlay(rgb_image, overlay_image, alpha=0.5):
    """Draw an overlay image on top of an RGB image."""
    return cv2.addWeighted(rgb_image, 1 - alpha, overlay_image, alpha, 0)


def draw_mask_overlay(rgb, mask, alpha=0.5, mask_color=(0, 255, 0), reduce_bg=False):
    """Draw a mask overlay on an RGB image.

    Args:
        rgb (np.ndarray): RGB image, shape (height, width, 3).
        mask (np.ndarray): Binary mask, shape (height, width).
        alpha (float): Transparency of the mask overlay.
        mask_color (tuple): RGB color of the mask overlay.
        reduce_bg (bool): If True, reduce the background intensity of the RGB image.

    Returns:
        np.ndarray: RGB image with mask overlay.
    """
    # Create an overlay based on whether to reduce the background
    overlay = np.zeros_like(rgb) if reduce_bg else rgb.copy()

    # Apply the mask color to the overlay where the mask is true
    overlay[mask.astype(bool)] = mask_color

    # Blend the overlay with the original image
    overlay = cv2.addWeighted(overlay, alpha, rgb, 1 - alpha, 0)

    return overlay


def draw_segmentation_mask_overlay(rgb_image, mask_image, alpha=0.5, reduce_bg=False):
    """Draw object masks overlayed on an RGB image."""
    overlay = np.zeros_like(rgb_image) if reduce_bg else rgb_image.copy()
    for label in np.unique(mask_image):
        if label == 0:
            continue
        color_idx = label % len(SEG_CLASS_COLORS)
        overlay[mask_image == label] = SEG_CLASS_COLORS[color_idx].rgb
    overlay = cv2.addWeighted(rgb_image, 1 - alpha, overlay, alpha, 0)
    return overlay


def read_pose_from_txt(file_path):
    """Read a pose matrix from a text file."""
    if not Path(file_path).exists():
        raise FileNotFoundError(f"Pose file '{file_path}' does not exist.")
    try:
        pose = np.loadtxt(str(file_path), dtype=np.float32)
    except Exception as e:
        raise ValueError(f"Failed to load pose from '{file_path}': {e}")
    return pose


def write_pose_to_txt(pose_path, pose, header="", fmt="%.8f"):
    """Write a pose matrix to a text file."""
    try:
        np.savetxt(str(pose_path), pose, fmt=fmt, header=header)
    except Exception as e:
        raise ValueError(f"Failed to write pose to '{pose_path}': {e}")


def extract_masks_from_labels(labels):
    masks = [(labels == i).astype(np.uint8) for i in np.unique(labels) if i != 0]
    return masks


def draw_annotated_image(rgb_image, boxes=None, masks=None, labels=None):
    """Annotate an RGB image with xyxy bboxes masks."""
    ann_imge = rgb_image.copy()

    if boxes is not None and masks is not None:
        assert len(boxes) == len(masks), "Boxes and masks must have the same length."
        class_ids = np.arange(len(boxes))
    elif boxes is not None:
        class_ids = np.arange(len(boxes))
    elif masks is not None:
        class_ids = np.arange(len(masks))
        boxes = [get_bbox_from_mask(m) for m in masks]
    else:
        return ann_imge

    # Detections
    detections = sv.Detections(
        xyxy=np.asarray(boxes, dtype=float),
        mask=np.asarray(masks, dtype=bool) if masks is not None else None,
        class_id=class_ids,
    )

    # Annotators
    annotators = []
    if boxes is not None:
        annotators.append(sv.BoxAnnotator())
    if masks is not None:
        annotators.append(sv.MaskAnnotator())

    # Apply all annotators
    for annotator in annotators:
        ann_imge = annotator.annotate(scene=ann_imge, detections=detections)

    # Add labels
    if labels is not None and len(labels) == len(boxes):
        label_annotator = sv.LabelAnnotator(text_position=sv.Position.TOP_CENTER)
        ann_imge = label_annotator.annotate(scene=ann_imge, detections=detections, labels=labels)

    return ann_imge


def draw_oriented_bbox_on_image(rgb_image, cam_K, ob_in_cam, bbox, line_color=(0, 255, 0), linewidth=2):
    """
    Draw an oriented 3D bounding box on a given RGB image.

    Args:
        rgb_image (np.ndarray): The RGB image as a (H, W, 3) NumPy array.
        cam_K (np.ndarray): Camera intrinsic matrix of shape (3, 3).
        ob_in_cam (np.ndarray): 4x4 transformation matrix from object to camera space.
        bbox (np.ndarray): (8, 3) array of 3D bounding box corners in object coordinates. Use below function to get it.
            mesh = trimesh.load("your_mesh.obj", process=False)
            bbox = mesh.bounding_box_oriented.corners  # (8, 3)
        line_color (tuple): RGB color for lines. Default is green.
        linewidth (int): Thickness of the bounding box lines.
    Returns:
        np.ndarray: Image with the bounding box drawn.
    """

    # Convert bbox to homogeneous coordinates
    bbox_homo = np.hstack([bbox, np.ones((8, 1))])  # (8, 4)

    # Transform to camera space
    bbox_cam = (ob_in_cam @ bbox_homo.T).T[:, :3]  # (8, 3)

    # Project to image space
    pts_2d = (cam_K @ bbox_cam.T).T  # (8, 3)
    pts_2d = pts_2d[:, :2] / pts_2d[:, 2:3]  # Normalize by Z
    pts_2d = np.round(pts_2d).astype(int)

    # Define cube edges
    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),  # bottom face
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),  # top face
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),  # sides
    ]

    # Draw lines
    img = rgb_image.copy()
    for start, end in edges:
        pt1 = tuple(pts_2d[start])
        pt2 = tuple(pts_2d[end])
        if (
            0 <= pt1[0] < img.shape[1]
            and 0 <= pt1[1] < img.shape[0]
            and 0 <= pt2[0] < img.shape[1]
            and 0 <= pt2[1] < img.shape[0]
        ):
            cv2.line(img, pt1, pt2, line_color, linewidth)

    return img
