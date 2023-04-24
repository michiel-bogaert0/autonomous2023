import json
from copy import copy
from dataclasses import dataclass
from pathlib import Path
from typing import List

import cv2
import numpy as np
import torch
import torchvision.transforms as T
from keypoint_detection.utils.heatmap import get_keypoints_from_heatmap
from keypoint_detection.utils.load_checkpoints import get_model_from_wandb_checkpoint
from torchvision.io import read_image
from tqdm import tqdm

CAT_TO_COLOUR = {
    0: (255, 0, 0),
    1: (0, 250, 250),
    2: (0, 100, 220),
    3: (0, 130, 220),
}


class TwoStageModel():
    def __init__(
        self,
        model,
        height_to_pos,
        n_channels=8,
        image_size=None,
        matching_threshold_px=50,
        min_cone_height=5,
    ):
        self.model = model
        self.height_to_pos = height_to_pos
        self.n_channels = n_channels

        self.matching_threshold_px = matching_threshold_px
        self.min_cone_height = min_cone_height

        # Warmup
        self.device = torch.device("cuda")
        self.model.to(self.device)
        self.model.eval()

        self.starter, self.ender = torch.cuda.Event(
            enable_timing=True
        ), torch.cuda.Event(enable_timing=True)
        self.timings = []
        self.image_size = image_size
        self.dummy_input = torch.rand(1, 3, *image_size).to(self.device)
        if image_size is not None:
            self.image_encoding = T.Resize((int(image_size[0]), int(image_size[1])))
        else:
            self.image_encoding = None

        for _ in range(50):
            result = self.model(self.dummy_input)

        # reset the timings
        self.warmup_timings = self.timings
        self.timings = []

    def run_bench(self, image):
        self.starter.record()
        result = self.model(image)
        self.ender.record()
        # WAIT FOR GPU SYNC
        torch.cuda.synchronize()
        self.timings.append(self.starter.elapsed_time(self.ender))
        return result

    def extract_cones(self, tops, bottoms, category: int):
        """Given a list of top and bottom keypoints extract the cone type and positions"""

        heights, valid_bottom_idx = np.empty(0), []
        for i, bottom in enumerate(bottoms):
            targets = tops[
                np.logical_and(tops[:, 0] > bottom[0], tops[:, 1] < bottom[1])
            ]
            if len(targets) == 0:
                continue
            diffs = targets - bottom
            errors = np.sum(np.power(diffs, 2), axis=1)
            best_match = np.argmin(errors)
            lowest_diff = diffs[best_match][0] ** 2 + diffs[best_match][1] ** 2

            threshold = self.matching_threshold_px**2
            if category == 2:
                threshold += 50**2
            if lowest_diff >= threshold:
                continue

            height = bottom[1] - targets[best_match, 1]
            if height < self.min_cone_height:
                continue

            heights = np.append(heights, height)
            valid_bottom_idx.append(i)

        if len(heights) == 0:
            return np.empty(0)

        pos = self.height_to_pos(category, heights, bottoms[valid_bottom_idx, :])
        cones = np.hstack([np.full((heights.shape[0], 1), category), pos])

        return cones

    def predict(self, original_image):
        image = torch.from_numpy(original_image).to(self.device).permute(2, 0, 1)
        # The image should be 3xHxW

        original_size = image.shape[1:]
        with torch.no_grad():
            if self.image_encoding is not None:
                image = self.image_encoding(
                    image
                )  # Don't forget to rescale the image before inference
            image = image[np.newaxis, ...].to(self.device).float() / 255
            result = self.run_bench(image)

        # List with elements: Array of #detectionsx2
        kpts = []
        for i in range(self.n_channels):
            arr = np.array(get_keypoints_from_heatmap(result[0, i], 2), dtype=float)

            if len(arr) == 0:
                arr = np.empty((0, 2))
            elif self.image_encoding is not None:
                # Don't forget to undo any image resizing afterwards
                arr[:, 0] *= original_size[1] / self.image_size[1]
                arr[:, 1] *= original_size[0] / self.image_size[0]
            kpts.append(arr)

        cones = np.empty((0, 4))
        for i in range(4):
            # If there are both top and bottom keypoints, run the processing code
            if kpts[2 * i].shape[0] > 0 and kpts[2 * i + 1].shape[0] > 0:
                extracted_cones = self.extract_cones(kpts[2 * i], kpts[2 * i + 1], i)
                if extracted_cones.size > 0:
                    cones = np.vstack((cones, extracted_cones))

        vis_img = np.copy(original_image)
        vis_img = cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR)
        vis_img = self.visualise(vis_img, kpts)
        vis_img = cv2.cvtColor(vis_img, cv2.COLOR_BGR2RGB)

        return cones, vis_img

    def visualise(self, image, kpts):
        for i in range(4):
            category = i
            keypoints = np.rint(np.vstack((kpts[2 * i], kpts[2 * i + 1]))).astype(int)
            for pred in keypoints:
                image = cv2.circle(image, pred, 3, CAT_TO_COLOUR[category], -1)
                image = cv2.circle(image, pred, 2, (0, 0, 0), -1)
                image = cv2.circle(image, pred, 1, (255, 255, 255), -1)

        return image
