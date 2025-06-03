from commons import *


# triangulate two points
def triangulate_point(uv1, uv2, P1, P2):
    """Triangulate a point from two views.

    Args:
        uv1: 2D point in view 1, shape (2,)
        uv2: 2D point in view 2, shape (2,)
        P1: Projection matrix for view 1, shape (3, 4)
        P2: Projection matrix for view 2, shape (3, 4)
    Returns:
        3D point
    """

    # Construct the A matrix
    A = np.zeros((4, 4))
    A[0] = uv1[0] * P1[2] - P1[0]
    A[1] = uv1[1] * P1[2] - P1[1]
    A[2] = uv2[0] * P2[2] - P2[0]
    A[3] = uv2[1] * P2[2] - P2[1]

    # Perform SVD
    _, _, V = np.linalg.svd(A)

    # The last row of V gives the solution
    X = V[-1]

    # Convert from homogeneous to 3D coordinates
    return X[:3] / X[3]


if __name__ == "__main__":
    uv1 = (358, 321)  # for camera 108222250342
    uv2 = (520, 257)  # for camera 046122250168

    # >>>>>>>>>> load the intrinsics <<<<<<<<<<<<
    K1 = read_K_matrix_from_json(
        PROJ_ROOT / "demo/calibration/intrinsics/108222250342_640x480.json"
    )
    K2 = read_K_matrix_from_json(
        PROJ_ROOT / "demo/calibration/intrinsics/046122250168_640x480.json"
    )

    # >>>>>>>>>> load the extrinsics and invert them <<<<<<<<<<<<
    RTs, master_camera = read_extrinsics_from_json(
        PROJ_ROOT / "demo/calibration/extrinsics/extrinsics_20231014/extrinsics.json"
    )
    RT1 = RTs["108222250342"]
    RT1_inv = np.linalg.inv(RT1)

    RT2 = RTs["046122250168"]
    RT2_inv = np.linalg.inv(RT2)

    # >>>>>>>>>> calculate the projection matrices <<<<<<<<<<<<
    P1 = K1 @ RT1_inv[:3, :]
    P2 = K2 @ RT2_inv[:3, :]

    # >>>>>>>>>> triangulate the points <<<<<<<<<<<<
    point_3d = triangulate_point(uv1, uv2, P1, P2)

    # >>>>>>>>>> visualize the triangulated point in 3D <<<<<<<<<<<<

    # read the color & depth images
    rgb1 = read_rgb_image(
        PROJ_ROOT / "demo/recordings/20231022_193630/108222250342/color_000000.jpg"
    )
    rgb2 = read_rgb_image(
        PROJ_ROOT / "demo/recordings/20231022_193630/046122250168/color_000000.jpg"
    )
    depth1 = read_depth_image(
        PROJ_ROOT / "demo/recordings/20231022_193630/108222250342/depth_000000.png"
    )
    depth2 = read_depth_image(
        PROJ_ROOT / "demo/recordings/20231022_193630/046122250168/depth_000000.png"
    )

    # convert the depth images to meters
    depth1 = depth1.astype(np.float32) / 1000.0
    depth2 = depth2.astype(np.float32) / 1000.0

    # get the normalized colors
    colors1 = rgb1.reshape(-1, 3) / 255.0
    colors2 = rgb2.reshape(-1, 3) / 255.0

    # backproject the depth to 3D
    points1 = deproject_depth_image(depth1, K1, RT1)
    points2 = deproject_depth_image(depth2, K2, RT2)

    # merge the points and colors
    merged_points = np.vstack([points1, points2])
    merged_colors = np.vstack([colors1, colors2])

    # create the point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(merged_points)
    pcd.colors = o3d.utility.Vector3dVector(merged_colors)

    # create the sphere at the triangulated point
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
    sphere.paint_uniform_color([1, 0, 0])  # red color
    sphere.translate(point_3d)

    # visualize the 3D points
    o3d.visualization.draw([pcd, sphere], point_size=1)
