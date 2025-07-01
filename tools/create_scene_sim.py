import rospy
from fetch_grasp.utils import PROJECT_ROOT
from fetch_grasp.utils.scene import ObjectService, SceneMaker
from fetch_grasp.utils.control import PointHeadClient, FollowTrajectoryClient

Z_OFFSET = -0.03  # difference between Real World and Gazebo table
TABLE_HEIGHT = 0.78 + Z_OFFSET

MODEL_NAMES = [
    "003_cracker_box",
    # "005_tomato_soup_can",
    "006_mustard_bottle",
    # "007_tuna_fish_can",
    # "008_pudding_box",
    # "009_gelatin_box",
    # "010_potted_meat_can",
    "011_banana",
    # "021_bleach_cleanser",
    # "024_bowl",
    # "025_mug",
    "035_power_drill",
    # "037_scissors",
    # "040_large_marker",
    # "052_extra_large_clamp",
]


def create_scene():
    rospy.loginfo("=" * 5 + " Creating scene in Gazebo... " + "=" * 5)
    # ---------- Add objects to the scene ----------
    sample_scene, _ = scene_m.create_scene()
    object_names = sorted(sample_scene.keys())

    cleanup_scene(object_names + ["cafe_table_org"])

    rospy.loginfo("Adding the cafe table...")
    objs.add_object("cafe_table_org", [*table_position, 0, 0, 0, 1])

    rospy.loginfo(f"Adding objects...")
    for obj_name in object_names:
        objs.add_object(obj_name, sample_scene[obj_name])
    rospy.sleep(3.0)  # wait for objects to be added
    print("Objects added to the scene!\n" + f"{object_names}")

    return sample_scene, object_names


def cleanup_scene(scene_objects):
    rospy.loginfo("Cleaning up the scene...")
    all_objects = scene_objects + ["cafe_table_org"]
    for obj in all_objects:
        try:
            objs.delete_object(obj)
        except Exception as e:
            pass


if __name__ == "__main__":
    pose_method = "gazebo"  # gazebo, fdpose
    models_dir = PROJECT_ROOT / "datasets/models"
    grasp_dir = PROJECT_ROOT / "datasets/grasp_data/refined_grasps"
    stable_pose_file = PROJECT_ROOT / "datasets/pose_data/selected_poses.pk"
    grid_size = (3, 3)
    table_position = [0.8, 0, Z_OFFSET]

    # ---------- Create a node ----------
    rospy.init_node("GazeboSceneGenerator")

    # ---------- Create services ----------
    objs = ObjectService(models_base_path=models_dir)
    scene_m = SceneMaker(MODEL_NAMES, models_dir, grid_size, table_position, TABLE_HEIGHT, stable_pose_file)

    # ---------- initialize clients for Fetch robot ----------
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()

    # Raise the torso
    rospy.loginfo("Raising torso...")
    torso_action.move_to([0.4])

    # look at table
    rospy.loginfo("Looking at the table...")
    if head_action.success:
        head_action.look_at(0.45, 0, TABLE_HEIGHT, "base_link")
    else:
        head_action = FollowTrajectoryClient("head_controller", ["head_pan_joint", "head_tilt_joint"])
        head_action.move_to([0.009195, 0.908270])

    create_scene()
