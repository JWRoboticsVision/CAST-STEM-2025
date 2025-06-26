import open3d as o3d
from open3d.visualization import gui
from open3d.visualization import rendering


from pathlib import Path
import numpy as np
import cv2
from ultralytics.models.sam import SAM, SAM2Predictor
import torch

from fetch_grasp.utils import PROJECT_ROOT
from fetch_grasp.utils.commons import (
    draw_annotated_image,
    read_rgb_image,
    write_rgb_image,
    write_depth_image,
    write_mask_image,
    write_data_to_yaml,
)


class MaskLabelToolkit:
    def __init__(self, device: str = "cpu", debug: bool = False) -> None:
        """
        Initializes the ImageLabelToolkit.

        Args:
            device (str): The device to run the model on ('cpu' or 'cuda').
            debug (bool): If True, enables debug mode with verbose logging.
        """
        self._device = device
        self._predictor = self._init_sam_predictor()

        self._save_folder = PROJECT_ROOT / "demo/ros"

        self._points = []
        self._undo_stack = []
        self._curr_mask = None
        self._curr_label = 0
        self._masks = []
        self._labels = []
        self._prompt_data = {}
        self._raw_image = None
        self._img_width = 640
        self._img_height = 480
        self._gui_image = o3d.geometry.Image(np.zeros((self._img_height, self._img_width, 3), dtype=np.uint8))
        self._text = ""
        self._is_done = False

    def _init_sam_predictor(self, model_type="vit_t"):
        """Initializes the SAM predictor."""
        predictor = SAM2Predictor(overrides={"device": self._device, "save": False})
        predictor.setup_model(model=SAM(f"{PROJECT_ROOT}/checkpoints/sam2.1_t.pt"))
        return predictor

    def run(self):
        """Runs the GUI application."""
        self._app = gui.Application.instance
        self._app.initialize()

        # Create window
        self._window = self._create_window()

        # Add callbacks
        self._window.set_on_layout(self._on_layout)
        self._window.set_on_close(self._on_close)
        self._window.set_on_key(self._on_key)
        self._widget3d.set_on_mouse(self._on_mouse_widget3d)

        if (self._save_folder / "color_image.png").exists():
            self._on_filedlg_done(str(self._save_folder / "color_image.png"))

        self._app.run()

    def _create_window(self, title: str = "Image Label Tool", width: int = 800, height: int = 600):
        """Creates the main GUI window."""
        window = gui.Application.instance.create_window(title=title, width=width, height=height)

        em = window.theme.font_size
        self._panel_width = 20 * em
        margin = 0.25 * em

        self._widget3d = gui.SceneWidget()
        # self._widget3d.enable_scene_caching(True)
        self._widget3d.scene = rendering.Open3DScene(window.renderer)
        self._widget3d.scene.set_background([1, 1, 1, 1], o3d.geometry.Image(self._gui_image))
        window.add_child(self._widget3d)

        self._info = gui.Label("")
        self._info.visible = False
        window.add_child(self._info)

        self._panel = gui.Vert(margin, gui.Margins(margin, margin, margin, margin))
        self._add_file_chooser_to_panel()
        self._add_buttons_to_panel()
        self._add_mask_label_to_panel()
        window.add_child(self._panel)

        # Widget Proxy
        self._proxy = gui.WidgetProxy()
        self._proxy.set_widget(None)
        self._panel.add_child(self._proxy)

        return window

    def _add_file_chooser_to_panel(self):
        """Adds file chooser to the panel."""
        self._fileedit = gui.TextEdit()
        filedlgbutton = gui.Button("...")
        filedlgbutton.horizontal_padding_em = 0.5
        filedlgbutton.vertical_padding_em = 0
        filedlgbutton.set_on_clicked(self._on_filedlg_button)
        fileedit_layout = gui.Horiz()
        fileedit_layout.add_child(gui.Label("Image file"))
        fileedit_layout.add_child(self._fileedit)
        fileedit_layout.add_fixed(0.25)
        fileedit_layout.add_child(filedlgbutton)
        self._panel.add_child(fileedit_layout)

    def _add_buttons_to_panel(self):
        """Adds buttons to the panel."""
        button_layout = gui.Horiz(0, gui.Margins(0.25, 0.25, 0.25, 0.25))
        addButton = gui.Button("Add Mask")
        removeButton = gui.Button("Remove Mask")
        saveButton = gui.Button("Save Mask")
        addButton.set_on_clicked(self._on_add_mask)
        removeButton.set_on_clicked(self._on_remove_mask)
        saveButton.set_on_clicked(self._on_save_mask)
        button_layout.add_stretch()
        button_layout.add_child(addButton)
        button_layout.add_stretch()
        button_layout.add_child(removeButton)
        button_layout.add_stretch()
        button_layout.add_child(saveButton)
        button_layout.add_stretch()
        self._panel.add_child(button_layout)

    def _add_mask_label_to_panel(self):
        """Adds mask label to the panel."""
        blk = gui.Horiz(0, gui.Margins(0, 0, 0, 0))
        blk.add_stretch()
        blk.add_child(gui.Label(f"---Current Mask---"))
        blk.add_stretch()
        blk.add_child(gui.Label(f"Label:"))
        self._intedit = gui.NumberEdit(gui.NumberEdit.INT)
        self._intedit.int_value = 0
        self._intedit.set_on_value_changed(self._on_intedit_changed)
        blk.add_child(self._intedit)
        blk.add_stretch()
        self._panel.add_child(blk)

    def _on_intedit_changed(self, value: int):
        """Handles changes in the mask label."""
        self._curr_label = int(value)

    def _mask_block(self):
        """Creates a block of mask labels."""
        if not self._labels:
            return None
        layout = gui.Vert(0, gui.Margins(0, 0, 0, 0))
        for idx, label in enumerate(self._labels):
            blk = gui.Horiz(0, gui.Margins(0, 0, 0, 0))
            blk.add_stretch()
            blk.add_child(gui.Label(f"Mask {idx}:"))
            blk.add_stretch()
            blk.add_child(gui.Label(f"Label: {label}"))
            blk.add_stretch()
            layout.add_child(blk)
        return layout

    def _on_layout(self, ctx):
        """Handles layout changes."""
        pref = self._info.calc_preferred_size(ctx, gui.Widget.Constraints())

        height = self._img_height

        self._widget3d.frame = gui.Rect(0, 0, self._img_width, height)
        self._panel.frame = gui.Rect(self._widget3d.frame.get_right(), 0, self._panel_width, height)
        self._info.frame = gui.Rect(
            self._widget3d.frame.get_left(),
            self._widget3d.frame.get_bottom() - pref.height,
            pref.width,
            pref.height,
        )
        self._window.size = gui.Size(self._img_width + self._panel_width, height)

    def _on_close(self):
        """Handles window close event."""
        self._is_done = True
        return True

    def _on_key(self, event):
        """Handles key events."""
        if event.key == gui.KeyName.Q:  # Quit
            if event.type == gui.KeyEvent.DOWN:
                self._window.close()
                return True

        if event.key == gui.KeyName.R:  # Reset points
            if event.type == gui.KeyEvent.DOWN:
                self._reset()
                return True

        return False

    def _on_add_mask(self):
        """Adds the current mask to the mask list."""
        self._masks.append(self._curr_mask)
        self._labels.append(self._intedit.int_value)
        self._prompt_data[self._intedit.int_value] = {
            "points": np.array(self._points)[:, :2].tolist(),
            "labels": np.array(self._points)[:, 2].tolist(),
        }
        self._curr_mask = None
        self._curr_label = 0
        self._reset()
        self._proxy.set_widget(self._mask_block())

    def _on_remove_mask(self):
        """Removes the last mask from the mask list."""
        if self._masks:
            self._masks.pop()
            self._labels.pop()
            self._proxy.set_widget(self._mask_block())

    def _on_save_mask(self):
        """Saves the current mask and its overlay."""
        self._save_folder.mkdir(parents=True, exist_ok=True)
        # Save mask
        mask, mask_vis = self.get_mask()
        write_mask_image(f"{self._save_folder}/mask_image_sam2.png", mask)
        # Save the mask visualization
        write_rgb_image(f"{self._save_folder}/mask_image_sam2_vis.png", mask_vis)
        # Save prompt data
        write_data_to_yaml(f"{self._save_folder}/sam2_prompts.yaml", self._prompt_data)

    def _on_filedlg_button(self):
        """Handles file dialog button click."""
        filedlg = gui.FileDialog(gui.FileDialog.OPEN, "Select file", self._window.theme)
        filedlg.add_filter(".png .jpg .jpeg", "Image files (*.png;*.jpg;*.jpeg)")
        filedlg.add_filter("", "All files")
        filedlg.set_on_cancel(self._on_filedlg_cancel)
        filedlg.set_on_done(self._on_filedlg_done)
        self._window.show_dialog(filedlg)

    def _on_filedlg_cancel(self):
        """Handles file dialog cancel."""
        self._window.close_dialog()

    def _update_image(self):
        """Updates the displayed image."""

        def update_image():
            self._widget3d.scene.set_background([1, 1, 1, 1], o3d.geometry.Image(self._gui_image))
            self._widget3d.force_redraw()

        self._app.post_to_main_thread(self._window, update_image)

    def _on_filedlg_done(self, path: str):
        """Handles file dialog done."""
        self._fileedit.text_value = path
        _path = Path(path).resolve()
        if not _path.exists():
            return
        img = cv2.imread(path)
        self._raw_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self._img_height, self._img_width, _ = self._raw_image.shape
        self._gui_image = o3d.geometry.Image(self._raw_image)
        self._predictor.set_image(img)
        self._masks = []
        self._labels = []
        self._curr_label = 0
        self._intedit.int_value = 0
        self._curr_mask = None
        self._reset()
        self._proxy.set_widget(self._mask_block())
        self._window.set_needs_layout()
        self._window.close_dialog()

    def _on_mouse_widget3d(self, event):
        """Handles mouse events in the 3D widget."""
        if event.type == gui.MouseEvent.Type.BUTTON_DOWN and event.is_modifier_down(gui.KeyModifier.CTRL):
            x = int(event.x - self._widget3d.frame.x)
            y = int(event.y - self._widget3d.frame.y)
            if event.buttons == gui.MouseButton.LEFT.value:
                prompt = (x, y, True)
            elif event.buttons == gui.MouseButton.RIGHT.value:
                prompt = (x, y, False)
            else:
                return gui.Widget.EventCallbackResult.IGNORED

            self._points.append(prompt)
            self._undo_stack.append(("add", prompt))
            self._update_sam_mask()
            curr_masks = self._masks + [self._curr_mask]
            curr_labels = [f"obj_{i}" for i in self._labels] + ["curr_obj"]
            curr_image = draw_annotated_image(self._raw_image, masks=curr_masks, labels=curr_labels)
            curr_image = self._draw_points(curr_image, self._points)
            self._gui_image = o3d.geometry.Image(curr_image)
            self._update_image()
            return gui.Widget.EventCallbackResult.HANDLED

        return gui.Widget.EventCallbackResult.IGNORED

    def _reset(self):
        """Resets the current points and mask."""
        self._points = []
        self._undo_stack = []
        # self._gui_image = o3d.geometry.Image(self._raw_image)
        self._update_image()

    def _undo_last_step(self):
        """Undoes the last action."""
        if self._undo_stack:
            action, data = self._undo_stack.pop()
            if action == "add":
                if self._points and self._points[-1] == data:
                    self._points.pop()
            self._update_sam_mask()
            curr_masks = self._masks + [self._curr_mask]
            curr_labels = [f"obj_{i}" for i in self._labels] + ["curr_obj"]
            curr_image = draw_annotated_image(self._raw_image, masks=curr_masks, labels=curr_labels)
            curr_image = self._draw_points(curr_image, self._points)
            self._gui_image = o3d.geometry.Image(curr_image)
            self._update_image()

    def _update_sam_mask(self):
        """Updates the SAM mask."""
        if self._points:
            result = self._predictor(
                points=np.array(self._points)[:, :2][None, ...],
                labels=np.array(self._points)[:, 2][None, ...],
            )[0]
            # self._curr_mask = result.masks.cpu().numpy().data[0].astype(np.uint8)
            self._curr_mask = result.masks.cpu().numpy().data[0]
        else:
            self._curr_mask = np.zeros(self._raw_image.shape[:2], dtype=np.uint8)

    def _draw_points(self, img: np.ndarray, points: list, is_rgb: bool = True) -> np.ndarray:
        """Draws points on the image."""
        img_copy = img.copy()
        for x, y, label in points:
            color = (0, 255, 0) if label == 1 else (0, 0, 255)
            if is_rgb:
                color = color[::-1]
            cv2.circle(img_copy, (x, y), 3, color, -1)
        return img_copy

    def get_mask(self) -> np.ndarray:
        """Gets the current mask."""
        mask = np.zeros(self._raw_image.shape[:2], dtype=np.uint8)

        if self._masks:
            for idx, m in enumerate(self._masks):
                mask[m > 0] = self._labels[idx]
            mask_vis = draw_annotated_image(
                self._raw_image, masks=self._masks, labels=[f"obj_{i}" for i in self._labels]
            )
        else:
            mask[self._curr_mask > 0] = self._curr_label
            self._prompt_data[self._curr_label] = {
                "points": np.array(self._points)[:, :2].tolist(),
                "labels": np.array(self._points)[:, 2].tolist(),
            }
            mask_vis = draw_annotated_image(self._raw_image, masks=[mask], labels=[f"obj_{self._curr_label}"])

        return mask, mask_vis


if __name__ == "__main__":
    device = "cuda" if torch.cuda.is_available() else "cpu"
    toolkit = MaskLabelToolkit(device=device)
    toolkit.run()
