import numpy as np
import trimesh
import pyrender
from pyrender.constants import RenderFlags

cvcam_in_glcam = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])


class OffscreenRenderer:
    def __init__(self, znear=0.01, zfar=10.0) -> None:
        self._znear = znear
        self._zfar = zfar
        self._cam_nodes = {}
        self._pyr_meshes = {}
        self._seg_colors = {}

    def __del__(self):
        self.clear_cameras()
        self.clear_meshes()

    def add_camera(self, cam_K, name):
        """Add a camera to the scene with its intrinsic matrix `cam_K`."""
        self._cam_nodes[name] = pyrender.Node(
            name=name,
            camera=pyrender.IntrinsicsCamera(
                fx=cam_K[0, 0],
                fy=cam_K[1, 1],
                cx=cam_K[0, 2],
                cy=cam_K[1, 2],
                znear=self._znear,
                zfar=self._zfar,
            ),
            light=pyrender.PointLight(color=[1.0, 1.0, 1.0], intensity=1.0),
        )

    def add_mesh(self, mesh, name, seg_color=(0, 0, 0)):
        """Add a mesh to the scene."""
        if isinstance(mesh, trimesh.Trimesh):
            pyr_mesh = pyrender.Mesh.from_trimesh(mesh)
        elif isinstance(mesh, str):
            pyr_mesh = pyrender.Mesh.from_trimesh(trimesh.load(mesh))
        else:
            raise ValueError("Invalid mesh type!!!")
        self._pyr_meshes[name] = pyr_mesh
        self._seg_colors[name] = seg_color

    def remove_camera(self, name):
        """Remove a camera from the scene."""
        if name in self._cam_nodes:
            del self._cam_nodes[name]

    def remove_mesh(self, name):
        """Remove a mesh from the scene."""
        if name in self._pyr_meshes:
            del self._seg_colors[name]
            del self._pyr_meshes[name]

    def clear_cameras(self):
        """Remove all cameras from the scene."""
        self._cam_nodes.clear()

    def clear_meshes(self):
        """Remove all meshes from the scene."""
        self._seg_colors.clear()
        self._pyr_meshes.clear()

    def _is_valid_pose(self, pose):
        if pose is None or not isinstance(pose, np.ndarray):
            return False
        return pose.shape == (4, 4) and not np.all(pose == -1)

    def _add_nodes_to_scene(self, scene, parent_node, node_dict, node_names, node_poses, seg_colors):
        seg_node_map = {}
        if isinstance(node_names, list):
            if len(node_names) != len(node_poses):
                raise ValueError("Mismatch between node_names and node_poses length")
            for name, pose in zip(node_names, node_poses):
                if name in node_dict and self._is_valid_pose(pose):
                    node = node_dict[name]
                    scene.add_node(node, parent_node=parent_node)
                    scene.set_pose(node, pose)
                    if name in seg_colors:
                        seg_node_map[node] = seg_colors[name]
        elif node_names in node_dict and self._is_valid_pose(node_poses):
            node = node_dict[node_names]
            scene.add_node(node, parent_node=parent_node)
            scene.set_pose(node, node_poses)
            if node_names in seg_colors:
                seg_node_map[node] = seg_colors[node_names]
        return seg_node_map

    def _create_scene(
        self,
        cam_names=None,
        cam_poses=None,
        mesh_names=None,
        mesh_poses=None,
        bg_color=[0, 0, 0, 1],
        ambient_light=[1.0, 1.0, 1.0, 1.0],
        seg_obj=False,
    ):
        seg_node_map = {}
        scene = pyrender.Scene(bg_color=bg_color, ambient_light=ambient_light)
        # Add world node to the scene
        world_node = scene.add_node(pyrender.Node(name="world"))
        # Add camera nodes to the scene
        if cam_names is not None and cam_poses is not None:
            if isinstance(cam_poses, list):
                glcam_poses = [pose @ cvcam_in_glcam for pose in cam_poses]
            else:
                glcam_poses = cam_poses @ cvcam_in_glcam
            self._add_nodes_to_scene(scene, world_node, self._cam_nodes, cam_names, glcam_poses, {})
        # Add mesh nodes to the scene
        if mesh_names is not None and mesh_poses is not None:
            mesh_nodes = {name: pyrender.Node(name=name, mesh=self._pyr_meshes[name]) for name in mesh_names}
            seg_node_map_o = self._add_nodes_to_scene(
                scene,
                world_node,
                mesh_nodes,
                mesh_names,
                mesh_poses,
                self._seg_colors,
            )
            if seg_obj:
                seg_node_map.update(seg_node_map_o)

        return scene, seg_node_map

    def _render_scene(
        self,
        width,
        height,
        cam_names,
        cam_poses,
        mesh_names,
        mesh_poses,
        bg_color,
        ambient_light,
        point_size,
        render_flags=RenderFlags.SKIP_CULL_FACES,
        seg_obj=False,
    ):
        """
        General method to render either colors or depths based on the provided parameters.

        Parameters:
        width, height (int): Dimensions of the render output.
        cam_names (str or list of str): Camera name(s).
        cam_poses (array or list of arrays): Camera pose(s).
        mesh_names (str or list of str, optional): Mesh name(s) to add.
        mesh_poses (array or list of arrays, optional): Mesh poses.
        bg_color, ambient_light (list, optional): Scene background color and ambient light.
        point_size (float, optional): Point size for rendering.
        render_flags (int, optional): Flags for rendering.
        seg_obj (bool, optional): Whether to render object and/or MANO segmentations.

        Returns:
        list of arrays: Rendered colors or depths for each camera view.
        """
        colors, depths = None, None

        r = pyrender.OffscreenRenderer(width, height, point_size)
        try:
            scene, seg_node_map = self._create_scene(
                cam_names,
                cam_poses,
                mesh_names,
                mesh_poses,
                bg_color,
                ambient_light,
                seg_obj,
            )
            # Render scene from each camera view (colors and depths)
            if isinstance(cam_names, list):
                colors, depths = [], []
                for cam_name in cam_names:
                    scene.main_camera_node = self._cam_nodes[cam_name]
                    color, depth = r.render(scene, render_flags, seg_node_map)
                    colors.append(color)
                    depths.append(depth)
            else:
                scene.main_camera_node = self._cam_nodes[cam_names]
                colors, depths = r.render(scene, render_flags, seg_node_map)
        finally:
            r.delete()

        return colors, depths

    def get_render_colors(
        self,
        width,
        height,
        cam_names,
        cam_poses,
        mesh_names=None,
        mesh_poses=None,
        bg_color=[0, 0, 0, 1],
        ambient_light=[1.0, 1.0, 1.0, 1.0],
        point_size=1.0,
    ):
        """
        Render colors from the scene based on the provided camera and mesh information.

        Returns:
        list of arrays: Rendered colors for each camera view.
        """
        colors, _ = self._render_scene(
            width,
            height,
            cam_names,
            cam_poses,
            mesh_names,
            mesh_poses,
            bg_color,
            ambient_light,
            point_size,
        )
        return colors

    def get_render_depths(
        self,
        width,
        height,
        cam_names,
        cam_poses,
        mesh_names=None,
        mesh_poses=None,
        bg_color=[0, 0, 0, 1],
        ambient_light=[1.0, 1.0, 1.0, 1.0],
        point_size=1.0,
    ):
        """
        Render depths from the scene based on the provided camera and mesh information.

        Returns:
        list of arrays: Rendered depths for each camera view.
        """
        _, depths = self._render_scene(
            width,
            height,
            cam_names,
            cam_poses,
            mesh_names,
            mesh_poses,
            bg_color,
            ambient_light,
            point_size,
        )
        return depths

    def get_render_segs(
        self,
        width,
        height,
        cam_names,
        cam_poses,
        mesh_names=None,
        mesh_poses=None,
        bg_color=[0, 0, 0, 1],
        ambient_light=[1.0, 1.0, 1.0, 1.0],
        point_size=1.0,
        seg_obj=True,
    ):
        """
        Render segmentations from the scene based on the provided camera and mesh information.

        Returns:
        list of arrays: Rendered segmentations for each camera view.
        """
        segs, _ = self._render_scene(
            width,
            height,
            cam_names,
            cam_poses,
            mesh_names,
            mesh_poses,
            bg_color,
            ambient_light,
            point_size,
            RenderFlags.SEG,
            seg_obj,
        )
        return segs
