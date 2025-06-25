import logging
import uuid

import cv2
import imageio
import numpy as np
import open3d as o3d
import torch
import torch.nn as nn
import torch.nn.functional as F
import nvdiffrast.torch as dr
from fetch_grasp.utils import make_clean_folder, add_path, PROJECT_ROOT

add_path(PROJECT_ROOT / "third-party")

# Import the FoundationPose module
from FoundationPose.Utils import (
    compute_mesh_diameter,
    euler_matrix,
    sample_views_icosphere,
    toOpen3dCloud,
    make_mesh_tensors,
    set_seed,
    depth2xyzmap,
    depth2xyzmap_batch,
    bilateral_filter_depth,
    erode_depth,
    mycpp,
    draw_posed_3d_box,
    draw_xyz_axis,
    set_logging_format,
)
from FoundationPose.learning.training.predict_score import ScorePredictor
from FoundationPose.learning.training.predict_pose_refine import PoseRefinePredictor

# Set logging level to CRITICAL to suppress unwanted messages
set_logging_format(logging.CRITICAL)

class FoundationPose:
    def __init__(
        self,
        model_pts,
        model_normals,
        symmetry_tfs=None,
        mesh=None,
        scorer: ScorePredictor = None,
        refiner: PoseRefinePredictor = None,
        glctx=None,
        debug=0,
        debug_dir=None,
        rotation_grid_min_n_views=40,
        rotation_grid_inplane_step=60,
        device="cuda:0",
    ):
        self.device = device
        self.gt_pose = None
        self.ignore_normal_flip = True
        self.debug = debug
        self.debug_dir = str(debug_dir)
        if self.debug >= 2:
            make_clean_folder(debug_dir)

        self.reset_object(model_pts, model_normals, symmetry_tfs=symmetry_tfs, mesh=mesh)
        self.make_rotation_grid(rotation_grid_min_n_views, rotation_grid_inplane_step)

        self.glctx = glctx
        self.scorer = scorer
        self.refiner = refiner

        self.pose_last = None  # Used for tracking; per the centered mesh

    def reset_object(self, model_pts, model_normals, symmetry_tfs=None, mesh=None):
        self.model_center = (model_pts.min(axis=0) + model_pts.max(axis=0)) / 2
        model_pts = model_pts - self.model_center.reshape(1, 3)

        if mesh is not None:
            self.mesh_ori = mesh.copy()
            mesh = mesh.copy()
            mesh.vertices = mesh.vertices - self.model_center.reshape(1, 3)

        self.diameter = float(compute_mesh_diameter(model_pts=model_pts))
        self.vox_size = max(self.diameter / 20.0, 0.003)
        logging.info(f"self.diameter:{self.diameter}, vox_size:{self.vox_size}")
        self.dist_bin = self.vox_size / 2
        self.angle_bin = 20  # Deg
        pcd = toOpen3dCloud(model_pts, normals=model_normals)
        pcd = pcd.voxel_down_sample(self.vox_size)
        self.max_xyz = np.asarray(pcd.points).max(axis=0)
        self.min_xyz = np.asarray(pcd.points).min(axis=0)
        self.pts = torch.tensor(np.asarray(pcd.points), dtype=torch.float).to(self.device)
        self.normals = F.normalize(torch.from_numpy(np.asarray(pcd.normals)).float().to(self.device), dim=-1)
        logging.info(f"self.pts:{self.pts.shape}")
        self.mesh_path = None
        self.mesh = mesh
        if self.mesh is not None:
            self.mesh_path = f"/tmp/{uuid.uuid4()}.obj"
            self.mesh.export(self.mesh_path)
        self.mesh_tensors = make_mesh_tensors(self.mesh, device=self.device)

        if symmetry_tfs is None:
            self.symmetry_tfs = torch.eye(4).float()[None].to(self.device)
        else:
            self.symmetry_tfs = torch.as_tensor(symmetry_tfs, device=self.device, dtype=torch.float)

        logging.info("reset done")

    def get_tf_to_centered_mesh(self):
        tf_to_center = torch.eye(4, dtype=torch.float, device=self.device)
        tf_to_center[:3, 3] = -torch.as_tensor(self.model_center, device=self.device, dtype=torch.float)
        return tf_to_center

    def to_device(self, s="cuda:0"):
        for k in self.__dict__:
            self.__dict__[k] = self.__dict__[k]
            if torch.is_tensor(self.__dict__[k]) or isinstance(self.__dict__[k], nn.Module):
                logging.info(f"Moving {k} to device {s}")
                self.__dict__[k] = self.__dict__[k].to(s)
        for k in self.mesh_tensors:
            logging.info(f"Moving {k} to device {s}")
            self.mesh_tensors[k] = self.mesh_tensors[k].to(s)
        if self.refiner is not None:
            self.refiner.model.to(s)
        if self.scorer is not None:
            self.scorer.model.to(s)
        if self.glctx is not None:
            self.glctx = dr.RasterizeCudaContext(s)
        self.device = s

    def make_rotation_grid(self, min_n_views=40, inplane_step=60):
        cam_in_obs = sample_views_icosphere(n_views=min_n_views)
        logging.info(f"cam_in_obs:{cam_in_obs.shape}")
        rot_grid = []
        for i in range(len(cam_in_obs)):
            for inplane_rot in np.deg2rad(np.arange(0, 360, inplane_step)):
                cam_in_ob = cam_in_obs[i]
                R_inplane = euler_matrix(0, 0, inplane_rot)
                cam_in_ob = cam_in_ob @ R_inplane
                ob_in_cam = np.linalg.inv(cam_in_ob)
                rot_grid.append(ob_in_cam)

        rot_grid = np.asarray(rot_grid)
        logging.info(f"rot_grid:{rot_grid.shape}")
        rot_grid = mycpp.cluster_poses(30, 99999, rot_grid, self.symmetry_tfs.data.cpu().numpy())
        rot_grid = np.asarray(rot_grid)
        logging.info(f"after cluster, rot_grid:{rot_grid.shape}")
        self.rot_grid = torch.as_tensor(rot_grid, device=self.device, dtype=torch.float)
        logging.info(f"self.rot_grid: {self.rot_grid.shape}")

    def generate_random_pose_hypo(self, K, rgb, depth, mask=None, scene_pts=None, init_ob_pos_center=None):
        """
        @scene_pts: torch tensor (N,3)
        """
        ob_in_cams = self.rot_grid.clone()
        if init_ob_pos_center is None:
            center = self.guess_translation(depth=depth, mask=mask, K=K)
        else:
            center = init_ob_pos_center.copy()
        ob_in_cams[:, :3, 3] = torch.tensor(center, device=self.device, dtype=torch.float).reshape(1, 3)
        return ob_in_cams

    def guess_translation(self, depth, mask, K):
        vs, us = np.where(mask > 0)
        if len(us) == 0:
            logging.info(f"mask is all zero")
            return np.zeros((3))
        uc = (us.min() + us.max()) / 2.0
        vc = (vs.min() + vs.max()) / 2.0
        valid = mask.astype(bool) & (depth >= 0.1)
        if not valid.any():
            logging.info(f"valid is empty")
            return np.zeros((3))

        zc = np.median(depth[valid])
        center = (np.linalg.inv(K) @ np.asarray([uc, vc, 1]).reshape(3, 1)) * zc

        if self.debug >= 2:
            pcd = toOpen3dCloud(center.reshape(1, 3))
            o3d.io.write_point_cloud(f"{self.debug_dir}/init_center.ply", pcd)

        return center.reshape(3)

    def register(self, K, rgb, depth, ob_mask=None, ob_id=None, glctx=None, iteration=5, init_ob_pos_center=None):
        """Copmute pose from given pts to self.pcd
        @pts: (N,3) np array, downsampled scene points
        """
        set_seed(0)
        logging.info("Welcome")

        if self.glctx is None:
            self.glctx = dr.RasterizeCudaContext() if glctx is None else glctx

        depth = erode_depth(depth, radius=2, device=self.device)
        depth = bilateral_filter_depth(depth, radius=2, device=self.device)

        if self.debug >= 2:
            xyz_map = depth2xyzmap(depth, K)
            valid = xyz_map[..., 2] >= 0.1
            pcd = toOpen3dCloud(xyz_map[valid], rgb[valid])
            o3d.io.write_point_cloud(f"{self.debug_dir}/scene_raw.ply", pcd)
            if ob_mask is not None:
                cv2.imwrite(f"{self.debug_dir}/ob_mask.png", (ob_mask * 255.0).clip(0, 255))

        normal_map = None
        if ob_mask is not None:
            valid = (depth >= 0.1) & (ob_mask > 0)
        else:
            valid = depth >= 0.1
        if valid.sum() < 100:
            depth = np.zeros_like(depth)

        if self.debug >= 2:
            imageio.imwrite(f"{self.debug_dir}/color.png", rgb)
            cv2.imwrite(f"{self.debug_dir}/depth.png", (depth * 1000).astype(np.uint16))
            valid = xyz_map[..., 2] >= 0.1
            pcd = toOpen3dCloud(xyz_map[valid], rgb[valid])
            o3d.io.write_point_cloud(f"{self.debug_dir}/scene_complete.ply", pcd)

        self.H, self.W = depth.shape[:2]
        self.K = K
        self.ob_id = ob_id
        self.ob_mask = ob_mask

        poses = self.generate_random_pose_hypo(
            K=K,
            rgb=rgb,
            depth=depth,
            mask=ob_mask,
            scene_pts=None,
            init_ob_pos_center=init_ob_pos_center,
        )
        poses = poses.data.cpu().numpy()

        if init_ob_pos_center is None:
            center = self.guess_translation(depth=depth, mask=ob_mask, K=K)
        else:
            center = init_ob_pos_center.copy()

        poses = torch.as_tensor(poses, device=self.device, dtype=torch.float)
        poses[:, :3, 3] = torch.as_tensor(center.reshape(1, 3), device=self.device)

        add_errs = self.compute_add_err_to_gt_pose(poses)
        logging.info(f"after viewpoint, add_errs min:{add_errs.min()}")

        xyz_map = depth2xyzmap(depth, K)
        poses, vis = self.refiner.predict(
            mesh=self.mesh,
            mesh_tensors=self.mesh_tensors,
            rgb=rgb,
            depth=depth,
            K=K,
            ob_in_cams=poses.data.cpu().numpy(),
            normal_map=normal_map,
            xyz_map=xyz_map,
            glctx=self.glctx,
            mesh_diameter=self.diameter,
            iteration=iteration,
            get_vis=self.debug >= 2,
        )
        if vis is not None:
            imageio.imwrite(f"{self.debug_dir}/vis_refiner.png", vis)

        scores, vis = self.scorer.predict(
            mesh=self.mesh,
            rgb=rgb,
            depth=depth,
            K=K,
            ob_in_cams=poses.data.cpu().numpy(),
            normal_map=normal_map,
            mesh_tensors=self.mesh_tensors,
            glctx=self.glctx,
            mesh_diameter=self.diameter,
            get_vis=self.debug >= 2,
        )
        if vis is not None:
            imageio.imwrite(f"{self.debug_dir}/vis_score.png", vis)

        add_errs = self.compute_add_err_to_gt_pose(poses)
        logging.info(f"final, add_errs min:{add_errs.min()}")

        ids = torch.as_tensor(scores).argsort(descending=True)
        logging.info(f"sort ids:{ids}")
        scores = scores[ids]
        poses = poses[ids]

        logging.info(f"sorted scores:{scores}")

        best_pose = poses[0] @ self.get_tf_to_centered_mesh()
        self.pose_last = poses[0].reshape(1, 4, 4)
        self.best_id = ids[0]

        return best_pose.data.cpu().numpy()

    def compute_add_err_to_gt_pose(self, poses):
        """
        @poses: wrt. the centered mesh
        """
        return -torch.ones(len(poses), device=self.device, dtype=torch.float)

    def track_one(self, rgb, depth, K, iteration, extra={}, prev_pose=None):
        if prev_pose is not None:
            logging.info("prev_pose is given")
            prev_pose_tensor = torch.from_numpy(prev_pose).unsqueeze(0).float().to(self.device)
            self.pose_last = prev_pose_tensor @ self.get_tf_to_centered_mesh().inverse()

        if self.pose_last is None:
            logging.info("Please init pose by register first")
            raise RuntimeError

        logging.info("Welcome")

        depth = torch.as_tensor(depth, device=self.device, dtype=torch.float)
        depth = erode_depth(depth, radius=2, device=self.device)
        depth = bilateral_filter_depth(depth, radius=2, device=self.device)
        logging.info("depth processing done")

        xyz_map = depth2xyzmap_batch(
            depth[None],
            torch.as_tensor(K, dtype=torch.float, device=self.device)[None],
            zfar=np.inf,
        )[0]

        pose, vis = self.refiner.predict(
            mesh=self.mesh,
            mesh_tensors=self.mesh_tensors,
            rgb=rgb,
            depth=depth,
            K=K,
            ob_in_cams=self.pose_last.data.cpu().numpy(),
            normal_map=None,
            xyz_map=xyz_map,
            mesh_diameter=self.diameter,
            glctx=self.glctx,
            iteration=iteration,
            get_vis=self.debug >= 2,
        )
        logging.info("pose done")
        if self.debug >= 2:
            extra["vis"] = vis

        self.pose_last = pose
        return (pose @ self.get_tf_to_centered_mesh()).data.cpu().numpy().reshape(4, 4)
