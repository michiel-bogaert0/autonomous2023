from typing import Tuple

import numpy as np
import numpy.typing as npt
import torch
import torchvision.transforms as T
from keypoint_detection.utils.heatmap import get_keypoints_from_heatmap
from keypoint_detection.utils.load_checkpoints import load_from_checkpoint


class KeypointDetector:
    def __init__(self, keypoint_model_path, detection_height_threshold, device) -> None:
        """Initialise a keypoint detector"""
        self.cone_image_margin = 5  # Padding that gets added to the top and left side before keypoint prediction
        self.keypoint_image_size = np.array([128, 96], dtype=int)
        self.detection_height_threshold = detection_height_threshold
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
            image: tensor on the GPU (3xHxW)
            bboxes: array of bounding boxes

        Returns:
            - cone validity mask
            - cone heights in pixels
            - bottom keypoint coordinates in pixels
        """

        # The image should also use pixel values between 0 and 1
        image = image.float() / 255

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
        with torch.no_grad():
            predicted_keypoints = self.model(cone_images)

        tops, bottoms = np.empty((0, 2)), np.empty((0, 2))
        final_cone_categories = np.empty(0)
        for i, pred in enumerate(predicted_keypoints):
            bottom = np.array(get_keypoints_from_heatmap(pred[1], 2))

            if len(bottom) == 0:
                # Without a bottom keypoint, we can't do anything
                valid_cones[i] = False
                continue

            if len(bottom) > 1:
                # If there are multiple bottoms, take the left one
                # The other one is probably the bottom right
                left_idx = np.argmin(bottom[:, 0])
                bottom = bottom[left_idx, :]
            else:
                bottom = bottom[0]

            # Rescale
            bottom[0] = (
                bottom[0] * cone_original_sizes[i][1] / self.keypoint_image_size[1]
                + cone_bbox_corners[i][0]
            )
            bottom[1] = (
                bottom[1] * cone_original_sizes[i][0] / self.keypoint_image_size[0]
                + cone_bbox_corners[i][1]
            )

            top = np.array(get_keypoints_from_heatmap(pred[0], 2))

            if len(top) == 0:
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
                if len(top) > 1:
                    # Multiple tops were found,
                    #   take the one closest to the top-center of the bbox
                    top_estimate = np.array(
                        (
                            self.cone_image_margin + self.keypoint_image_size[0] / 2,
                            self.cone_image_margin,
                        )
                    )
                    errors = top - top_estimate
                    dist = np.sum(np.power(errors, 2), axis=1)
                    print(errors, dist)

                    top = top[np.argmin(dist), :]
                else:
                    top = top[0]

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
