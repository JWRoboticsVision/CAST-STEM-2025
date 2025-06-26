import numpy as np
import torch

from fetch_grasp.utils import PROJECT_ROOT
from fetch_grasp.utils.commons import (
    read_rgb_image,
    write_rgb_image,
    write_mask_image,
    draw_annotated_image,
    extract_masks_from_labels,
    write_data_to_yaml,
)

OBJECT_CLASSES = [
    "background",
    "002_master_chef_can",
    "003_cracker_box",
    "004_sugar_box",
    "005_tomato_soup_can",
    "006_mustard_bottle",
    "007_tuna_fish_can",
    "008_pudding_box",
    "009_gelatin_box",
    "010_potted_meat_can",
    "011_banana",
    "019_pitcher_base",
    "021_bleach_cleanser",
    "024_bowl",
    "025_mug",
    "035_power_drill",
    "036_wood_block",
    "037_scissors",
    "040_large_marker",
    "051_large_clamp",
    "052_extra_large_clamp",
    "061_foam_brick",
]


def run_nidsnet_once(nids_mod, image_rgb):
    _, labels = nids_mod.step(image_rgb)
    labels = labels.cpu().numpy().astype(np.uint8)
    masks = extract_masks_from_labels(labels)
    obj_names = [OBJECT_CLASSES[int(i)] for i in np.unique(labels) if i != 0]
    labels_vis = draw_annotated_image(image_rgb, masks=masks, labels=obj_names)
    return masks, obj_names, labels, labels_vis


def initialize_nidsnet():
    from fetch_grasp.wrappers.nidsnet import NIDS, feat_dict, weight_adapter_path

    object_features = torch.Tensor(feat_dict["features"]).view(-1, 42, 1024).cuda()
    model = NIDS(template_features=object_features, use_adapter=True, adapter_path=weight_adapter_path)
    return model


if __name__ == "__main__":
    # Directories
    models_dir = f"{PROJECT_ROOT}/third-party/SceneReplica/Datasets/benchmarking/models"
    save_dir = PROJECT_ROOT / "demo/ros"

    cam_K_file = save_dir / "cam_K.txt"
    color_file = save_dir / "color_image.png"

    # Initialize NIDS-Net
    nids_model = initialize_nidsnet()

    # Load cam_K and color image
    cam_K = np.loadtxt(cam_K_file, dtype=np.float32)
    rgb = read_rgb_image(color_file)

    # Run NIDS-Net Inference
    masks, obj_names, labels, labels_vis = run_nidsnet_once(nids_model, rgb)

    # Save results
    write_mask_image(f"{save_dir}/mask_image_nidsnet.png", labels)
    write_rgb_image(f"{save_dir}/mask_image_nidsnet_vis.png", labels_vis)
    write_data_to_yaml(
        f"{save_dir}/nidsnet_class_names.yaml", {int(i): OBJECT_CLASSES[int(i)] for i in np.unique(labels) if i != 0}
    )
