from commons import *


class SequenceLoader:
    def __init__(self, sequence_folder) -> None:
        self.data_folder = Path(sequence_folder).resolve()
        # calibration folder
        self.calib_folder = self.data_folder.parent.parent / "calibration"

        # load metadata
        self.load_metadata()

        # load intrinsics
        self.load_intrinsics()

        # load extrinsics
        self.load_extrinsics()

    def load_metadata(self):
        meta_file = self.data_folder / "meta.json"
        data = read_data_from_json(meta_file)
        self.rs_serials = data["realsense"]["serials"]
        self.rs_width = data["realsense"]["width"]
        self.rs_height = data["realsense"]["height"]
        self.extr_file = (
            self.calib_folder
            / "extrinsics"
            / data["calibration"]["extrinsics"]
            / "extrinsics.json"
        )
        self.subject_id = data["calibration"]["mano"]
        self.mano_file = self.calib_folder / "mano" / self.subject_id / "mano.json"
        self.object_ids = data["object_ids"]
        self.mano_sides = data["mano_sides"]
        self.num_frames = data["num_frames"]

    def load_intrinsics(self):
        self.intrinsics = [
            read_K_matrix_from_json(
                self.calib_folder
                / "intrinsics"
                / f"{s}_{self.rs_width}x{self.rs_height}.json"
            )
            for s in self.rs_serials
        ]

    def load_extrinsics(self):
        extrinsics, rs_master = read_extrinsics_from_json(self.extr_file)
        self.rs_master = rs_master

        tag1 = extrinsics["tag_1"]
        tag1_inv = np.linalg.inv(tag1)

        self.extrinsics2master = np.stack([extrinsics[s] for s in self.rs_serials])
        self.extrinsics2master_inv = np.stack(
            [np.linalg.inv(t) for t in self.extrinsics2master], axis=0
        )

        # the loaded extrinsics are actually from the master to the slave
        # we need to multiply by the tag1_inv to get the slave to world
        self.extrinsics2world = np.stack(
            [tag1_inv @ extrinsics[s] for s in self.rs_serials], axis=0
        )
        self.extrinsics2world_inv = np.stack(
            [np.linalg.inv(t) for t in self.extrinsics2world], axis=0
        )

    def get_rgb_image(self, frame_id, serial):
        img_file = self.data_folder / serial / f"color_{frame_id:06d}.jpg"
        if not img_file.exists():
            return np.zeros((self.rs_height, self.rs_width, 3), dtype=np.uint8)
        return read_rgb_image(img_file)

    def get_depth_image(self, frame_id, serial):
        img_file = self.data_folder / serial / f"depth_{frame_id:06d}.png"
        if not img_file.exists():
            return np.zeros((self.rs_height, self.rs_width), dtype=np.uint16)
        return read_depth_image(img_file)

    def get_points_camera(self, frame_id, serial):
        depth = self.get_depth_image(frame_id, serial)
        depth = depth.astype(np.float32) / 1000.0
        K = self.intrinsics[self.rs_serials.index(serial)]
        RT = self.extrinsics2master[self.rs_serials.index(serial)]
        points = deproject_depth_image(depth, K, RT)
        return points

    def get_points_world(self, frame_id, serial):
        depth = self.get_depth_image(frame_id, serial)
        depth = depth.astype(np.float32) / 1000.0
        K = self.intrinsics[self.rs_serials.index(serial)]
        RT = self.extrinsics2world[self.rs_serials.index(serial)]
        points = deproject_depth_image(depth, K, RT)
        return points


if __name__ == "__main__":
    sequence_folder = PROJ_ROOT / "demo/recordings/20231022_193630"
    loader = SequenceLoader(sequence_folder)
    rs_serials = loader.rs_serials

    frame_id = 0

    # load rgb and depth images for all realsense cameras
    rgb_images = [loader.get_rgb_image(frame_id, s) for s in rs_serials]
    depth_images = [loader.get_depth_image(frame_id, s) for s in rs_serials]

    # display_images
    display_images(rgb_images + depth_images, rs_serials + rs_serials)

    # prepare colors for point cloud
    colors = np.vstack(
        [img.reshape(-1, 3).astype(np.float32) / 255.0 for img in rgb_images]
    )

    # get points in camera coordinates
    points_camera = np.vstack(
        [loader.get_points_camera(frame_id, s) for s in rs_serials]
    )
    # display point cloud
    pcd_c = o3d.geometry.PointCloud()
    pcd_c.points = o3d.utility.Vector3dVector(points_camera)
    pcd_c.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw([pcd_c])

    # get points in world coordinates
    points_world = np.vstack([loader.get_points_world(frame_id, s) for s in rs_serials])
    # display point cloud
    pcd_w = o3d.geometry.PointCloud()
    pcd_w.points = o3d.utility.Vector3dVector(points_world)
    pcd_w.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw([pcd_w])
