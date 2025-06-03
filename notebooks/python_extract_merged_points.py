from commons import *

CAMERA_IDS = (
    "037522251142",
    "043422252387",
    "046122250168",
    "105322251225",
    "105322251564",
    "108222250342",
    "115422250549",
    "117222250549",
)


def extract_merged_points():
    # load extrinsics
    RTs, master_cam = read_extrinsics_from_json(
        PROJ_ROOT / f"demo/calibration/extrinsics/extrinsics_20231014/extrinsics.json"
    )

    merged_points = []
    merged_colors = []
    for camera_id in CAMERA_IDS:
        # camera intrinsics
        K = read_K_matrix_from_json(
            PROJ_ROOT / f"demo/calibration/intrinsics/{camera_id}_640x480.json"
        )
        # camera extrinsics
        RT = RTs[camera_id]
        RT_inv = np.linalg.inv(RT)  # inverse of extrinsics

        # load RGBD images
        rgb = read_rgb_image(
            PROJ_ROOT / f"demo/recordings/20231022_193630/{camera_id}/color_000000.jpg"
        )
        depth = read_depth_image(
            PROJ_ROOT / f"demo/recordings/20231022_193630/{camera_id}/depth_000000.png"
        )

        # convert depth to meters
        depth = depth.astype(np.float32) / 1000.0

        # get points in world frame
        points = deproject_depth_image(depth, K, RT)

        # get colors
        colors = rgb.reshape(-1, 3).astype(np.float32) / 255.0

        merged_points.append(points)
        merged_colors.append(colors)

    # convert to numpy arrays
    # merged_points = np.concatenate(merged_points, axis=0)
    # merged_colors = np.concatenate(merged_colors, axis=0)
    merged_points = np.vstack(merged_points)
    merged_colors = np.vstack(merged_colors)

    return merged_points, merged_colors


if __name__ == "__main__":
    merged_points, merged_colors = extract_merged_points()

    # create point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(merged_points)
    pcd.colors = o3d.utility.Vector3dVector(merged_colors)

    # visualize
    o3d.visualization.draw([pcd], point_size=1)
