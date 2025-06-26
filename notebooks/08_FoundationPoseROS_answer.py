import numpy as np
import cv2
import open3d as o3d

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

from fetch_grasp.utils import PROJECT_ROOT
from fetch_grasp.utils.commons import write_rgb_image, write_depth_image, colorize_depth_image, deproject_depth_image


def get_camera_K(caminfo_topic):
    camInfo_msg = rospy.wait_for_message(caminfo_topic, CameraInfo)
    cam_model = PinholeCameraModel()
    cam_model.fromCameraInfo(camInfo_msg)
    K = cam_model.intrinsicMatrix().astype("float32")
    img_height, img_width = cam_model.height, cam_model.width

    return K, img_height, img_width


if __name__ == "__main__":
    # ---------- Prerequisites ----------
    # Initialize the ROS node
    fdpose_node = rospy.init_node("fdpose_node", anonymous=False)

    # Initialize CvBridge
    bridge = CvBridge()

    # Ouptut directory
    save_dir = PROJECT_ROOT / "datasets/tmp"
    save_dir.mkdir(parents=True, exist_ok=True)

    # Topics to subscribe to
    color_topic = "/head_camera/rgb/image_raw"
    depth_opic = "/head_camera/depth_registered/image_raw"
    caminfo_topic = "/head_camera/rgb/camera_info"

    # Subscribe to Color and Depth image topics
    # hint: use rospy.wait_for_message() function to get the color and depth images
    color_msg = rospy.wait_for_message(color_topic, Image)
    depth_msg = rospy.wait_for_message(depth_opic, Image)

    # Convert the Image messages to OpenCV images
    # hint: use CvBridge to convert the messages, and convert the color image to RGB format
    color_img = bridge.imgmsg_to_cv2(color_msg, desired_encoding="passthrough")
    depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

    # * Convert the color image to RGB format
    # hint: use cv2.cvtColor() function to convert the color image from BGR to RGB
    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

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
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)  # Normalize colors to [0, 1]
    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])
