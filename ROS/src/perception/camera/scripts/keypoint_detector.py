from typing import Tuple

import numpy as np
import torch
import torchvision.transforms as T
from keypoint_detection.utils.load_checkpoints import load_from_checkpoint


class KeypointDetector:
    def __init__(self, keypoint_model_path, device) -> None:
        """Initialise a keypoint detector"""
        # Use this 5px margin, since the kpt detector is trained on this
        # Basically it prevents edge artefacts during kpt inference
        self.cone_image_margin = 5
        self.keypoint_heatmap_threshold = 0.1  # The minimal heatmap intensity value that can be accepted as a keypoint
        # Use this (128x96, hxw) cropped cone size since the kpt detector is trained on this
        self.keypoint_image_size = np.array([128, 96], dtype=int)
        self.image_resizer = T.Resize(
            size=(self.keypoint_image_size[0], self.keypoint_image_size[1]),
            antialias=False,
        )

        self.device = device

        self.model = load_from_checkpoint(str(keypoint_model_path))
        self.model = self.model.to(self.device)
        self.model.eval()

        # Warm-up
        print("Keypoint warm-up")
        self.dummy_input = torch.rand(20, 3, *self.keypoint_image_size).to(self.device)
        for _ in range(30):
            self.model(self.dummy_input)

    def predict(
        self, image: torch.Tensor, bboxes
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Given an image and YOLO bounding boxes, predict keypoints on each cone in the image

        Args:
            image: normalised tensor on the GPU (3xHxW)
            bboxes: array of bounding boxes

        Returns:
            - cone validity mask
            - cone heights in pixels
            - bottom keypoint coordinates in pixels
        """

        valid_cones = np.ones(bboxes.shape[0], dtype=bool)

        cone_original_sizes, cone_bbox_corners = np.empty((0, 2)), np.empty((0, 2))
        cone_images = torch.empty((bboxes.shape[0], 3, *self.keypoint_image_size)).to(
            self.device
        )
        for i in range(bboxes.shape[0]):
            xyxy = bboxes[i, :4].astype(int)
            cone_img = image[:, xyxy[1] : xyxy[3], xyxy[0] : xyxy[2]]
            original_size = cone_img.shape[1:]
            resized_image = self.image_resizer(cone_img)

            # Batch all detected cones
            cone_original_sizes = np.vstack((cone_original_sizes, original_size))
            cone_bbox_corners = np.vstack((cone_bbox_corners, xyxy[:2]))
            cone_images[i] = resized_image

        # Infer batched keypoints
        with torch.inference_mode():
            predicted_keypoints = self.model(cone_images)

        tops, bottoms = np.empty((0, 2)), np.empty((0, 2))
        predicted_keypoints = predicted_keypoints.cpu().numpy()

        # Iterate over all predicted keypoint and pick out the good pairs
        # The keypoints for each cropped cone are chosen using a (filtered) argmax on the heatmap
        for i, pred in enumerate(predicted_keypoints):
            # Find a bottom keypoint
            bottom_idx = np.argmax(pred[1])
            y, x = divmod(bottom_idx, self.keypoint_image_size[1])
            bottom = np.array([x, y])

            if pred[1][y, x] < self.keypoint_heatmap_threshold:
                # Without a bottom keypoint, we can't do anything
                valid_cones[i] = False
                continue

            # Rescale
            bottom[0] = (
                bottom[0] * cone_original_sizes[i][1] / self.keypoint_image_size[1]
                + cone_bbox_corners[i][0]
            )
            bottom[1] = (
                bottom[1] * cone_original_sizes[i][0] / self.keypoint_image_size[0]
                + cone_bbox_corners[i][1]
            )

            # Find a top keypoint
            top_idx = np.argmax(pred[0])
            y, x = divmod(top_idx, self.keypoint_image_size[1])
            top = np.array([x, y])

            if pred[0][y, x] < self.keypoint_heatmap_threshold:
                # If no top was detected, we use the estimate
                top = np.array(
                    (
                        cone_bbox_corners[i][0]
                        + self.cone_image_margin
                        + cone_original_sizes[i][1] / 2,
                        cone_bbox_corners[i][1] + self.cone_image_margin,
                    )
                )
            else:
                # Rescale
                top[0] = (
                    top[0] * cone_original_sizes[i][1] / self.keypoint_image_size[1]
                    + cone_bbox_corners[i][0]
                )
                top[1] = (
                    top[1] * cone_original_sizes[i][0] / self.keypoint_image_size[0]
                    + cone_bbox_corners[i][1]
                )
            tops = np.vstack((tops, top))
            bottoms = np.vstack((bottoms, bottom))

        cone_heights = bottoms[:, 1] - tops[:, 1]

        return valid_cones, cone_heights, bottoms
