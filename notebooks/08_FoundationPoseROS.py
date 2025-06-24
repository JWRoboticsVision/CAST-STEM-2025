from pathlib import Path
import numpy as np
import cv2
from matplotlib import colormaps
import open3d as o3d

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

from fetch_grasp.utils import PROJECT_ROOT
from fetch_grasp.utils.commons import write_rgb_image, write_depth_image


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


def get_camera_K(caminfo_topic):
    # initialize the PinholeCameraModel
    cam_model = PinholeCameraModel()

    # get the camera info message
    camInfo_msg = rospy.wait_for_message(...)  # complete this line

    # set the camera model from the CameraInfo message
    # hint: https://docs.ros.org/en/noetic/api/image_geometry/html/python/index.html#image_geometry.PinholeCameraModel.fromCameraInfo

    # get intrinsics from the camera model
    K = ...  # complete this line

    # get image dimensions from the camera model
    img_height, img_width = cam_model.height, cam_model.width

    return K, img_height, img_width


if __name__ == "__main__":
    # ---------- Prerequisites ----------
    # Initialize the ROS node
    fdpose_node = rospy.init_node("fdpose_node", anonymous=False)

    # Initialize CvBridge
    bridge = CvBridge()

    # Ouptut directory
    save_dir = PROJECT_ROOT / "outputs"
    save_dir.mkdir(parents=True, exist_ok=True)

    # Topics to subscribe to
    color_topic = "/head_camera/rgb/image_raw"
    depth_opic = "/head_camera/depth_registered/image_raw"
    caminfo_topic = "/head_camera/rgb/camera_info"

    # Subscribe to Color and Depth image topics
    # hint: use rospy.wait_for_message() function to get the color and depth images
    color_msg = rospy.wait_for_message(...)  # complete this line
    depth_msg = rospy.wait_for_message(...)  # complete this line

    # Convert the Image messages to OpenCV images
    # hint: use CvBridge to convert the messages, and convert the color image to RGB format
    color_img = bridge.imgmsg_to_cv2(...)  # complete this line
    depth_img = bridge.imgmsg_to_cv2(...)  # complete this line

    # * Convert the color image to RGB format
    # hint: use cv2.cvtColor() function to convert the color image from BGR to RGB
    color_img = cv2.cvtColor(...)  # complete this line

    # Colorize the depth image for visualization
    depth_img_vis = colorize_depth_image(depth_img)

    # The subscribed depth image dtype is float32, which is in meters.
    # Convert depth image to millimeters and change dtype to uint16 for saving
    depth_img = (depth_img * 1000.0).astype(np.uint16)

    # Save the images to disk
    write_rgb_image(save_dir / "color_image.png", color_img)
    write_depth_image(save_dir / "depth_image.png", depth_img)
    write_rgb_image(save_dir / "depth_image_vis.png", depth_img_vis)

    # Get camera info
    # complete the get_camera_K function to retrieve the camera intrinsic parameters
    # and image dimensions from the CameraInfo message
    K, img_height, img_width = get_camera_K(caminfo_topic)

    # Save camera intrinsic parameters
    np.savetxt(save_dir / "cam_K.txt", K, fmt="%.6f")

    # Deproject the depth image to 3D points
    depth = depth_img.astype(np.float32) / 1000.0  # Convert to meters
    points = deproject_depth_image(depth, K)
    colors = color_img.reshape(-1, 3).astype(np.float32) / 255.0  # Normalize colors to [0, 1]
    valid_mask = (points[:, 2] > 0) & (points[:, 2] < 1.0)  # Filter out invalid points
    points = points[valid_mask]
    colors = colors[valid_mask]

    # Create Open3D point cloud
    # hint: https://www.open3d.org/docs/release/python_example/geometry/point_cloud/index.html#point-cloud-with-numpy-py
    pcd = o3d.geometry.PointCloud()
    pcd.points = ...
    pcd.colors = ...

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])
