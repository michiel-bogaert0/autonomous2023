from typing import Tuple

import numpy as np
import numpy.typing as npt
import torch
import torchvision.transforms as T
from keypoint_detection.utils.heatmap import get_keypoints_from_heatmap
from keypoint_detection.utils.load_checkpoints import load_from_checkpoint


class KeypointDetector:
    def __init__(self, checkpoint_path, detection_height_threshold, device) -> None:
        """Initialise a keypoint detector"""
        self.cone_image_margin = 5  # Padding that gets added to the top and left side before keypoint prediction
        self.keypoint_image_size = np.array([128, 96])
        self.detection_height_threshold = detection_height_threshold
        self.image_resizer = T.Resize(
            size=(int(self.keypoint_image_size[0]), int(self.keypoint_image_size[1]))
        )

        self.device = device

        self.model = load_from_checkpoint(checkpoint_path)
        self.model = self.model.to(self.device)
        self.model.eval()

    def predict(
        self, image: torch.Tensor, bboxes
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Given an image and YOLO bounding boxes, predict keypoints on each cone in the image

        Args:
            image: tensor on the GPU (HxWx3)
            bboxes: YOLOv8 'boxes' object

        Returns:
            - cone categories
            - cone heights in pixels
            - bottom keypoint coordinates in pixels
        """

        # The image should be 3xHxW
        image = image.permute(2, 0, 1)
        # The image should also use pixel values between 0 and 1
        image = image.float() / 255

        prelim_cone_categories = []
        cone_original_sizes = []
        cone_bbox_corners = []
        cone_images = torch.empty((0, 3, *self.keypoint_image_size)).to(self.device)
        for bbox in bboxes:
            category = bbox.cls.int().detach().cpu().item()

            # Format:
            #  Top-left: u, v
            #  Bottom-right: u, v
            bbox_kpts = [round(x) for x in bbox.xyxy[-1].detach().cpu().numpy()]
            bbox_kpts[0] = max(0, bbox_kpts[0] - self.cone_image_margin)
            bbox_kpts[1] = max(0, bbox_kpts[1] - self.cone_image_margin)

            if bbox_kpts[3] - bbox_kpts[1] < self.detection_height_threshold:
                # Detection is not tall enough
                continue
            if bbox_kpts[2] - bbox_kpts[0] < self.detection_height_threshold / 4:
                # Detection not wide enough
                continue

            resized_image = image[
                :, bbox_kpts[1] : bbox_kpts[3], bbox_kpts[0] : bbox_kpts[2]
            ]
            original_size = resized_image.shape[1:]
            resized_image = self.image_resizer(resized_image)
            # Batch all detected cones
            prelim_cone_categories.append(category)
            cone_original_sizes.append(original_size)
            cone_bbox_corners.append((bbox_kpts[0], bbox_kpts[1]))
            cone_images = torch.vstack((cone_images, resized_image.unsqueeze(0)))

        # Infer batched keypoints
        with torch.no_grad():
            predicted_keypoints = self.model(cone_images)

        tops, bottoms = torch.empty((0, 2)), torch.empty((0, 2))
        final_cone_categories = []
        for i, pred in enumerate(predicted_keypoints):
            bottom = torch.tensor(
                get_keypoints_from_heatmap(
                    pred[1], min_keypoint_pixel_distance=2, max_keypoints=2
                )
            )

            if len(bottom) == 0:
                # Without a bottom keypoint, we can't do anything
                continue

            if len(bottom) > 1:
                # If there are multiple bottoms, take the left one
                # The other one is probably the bottom right
                left_idx = torch.argmin(bottom[:, 0])
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

            top = torch.tensor(get_keypoints_from_heatmap(pred[0], 2))

            if len(top) == 0:
                # If no top was detected, we use the estimate
                top = torch.tensor(
                    (
                        cone_bbox_corners[i][0]
                        + self.cone_image_margin
                        + cone_original_sizes[i][1] / 2,
                        cone_bbox_corners[i][1] + self.cone_image_margin,
                    )
                )
            else:
                if len(top) > 1:
                    raise NotImplementedError("Cannot handle multiple top keypoints")

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

            tops = torch.vstack((tops, top))
            bottoms = torch.vstack((bottoms, bottom))
            final_cone_categories.append(prelim_cone_categories[i])

        cone_heights = bottoms[:, 1] - tops[:, 1]

        return torch.tensor(final_cone_categories), cone_heights, bottoms
