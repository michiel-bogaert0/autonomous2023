import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import numpy.typing as npt
import torch
import torchvision.transforms as T
from keypoint_detection.utils.load_checkpoints import load_from_checkpoint

CAT_TO_COLOUR = {
    0: (255, 0, 0),
    1: (0, 250, 250),
    2: (0, 100, 220),
    3: (0, 130, 220),
}


class TwoStageModel:
    def __init__(
        self,
        keypoint_model_path,
        height_to_pos,
        image_size=None,
        matching_threshold_px=150,
        detection_height_threshold=33,
        device="cuda:0",
        visualise=False,
    ):
        self.height_to_pos = height_to_pos

        self.matching_threshold_px = matching_threshold_px
        self.detection_height_threshold = detection_height_threshold
        self.use_vis = visualise

        # Warmup
        self.device = device
        self.model = load_from_checkpoint(str(keypoint_model_path))
        self.model.to(self.device)
        self.model.eval()

        self.image_size = image_size
        if image_size is not None:
            self.image_encoding = T.Resize(image_size, antialias=False)
        else:
            self.image_encoding = None

        print("Keypoint warm-up")
        self.dummy_input = torch.rand(1, 3, *image_size).to(self.device)
        for _ in range(50):
            _ = self.model(self.dummy_input)

    def extract_cones(self, tops, bottoms, category: int):
        """Given a list of top and bottom keypoints extract the cone type and positions

        Args:
            tops: top keypoint positions (u, v)
            bottoms: bottom keypoint positions (u, v)
            category: the cone type

        Returns:
            an array of Nx4 containing (cone category, x, y, z)
        """

        heights, valid_bottom_idx = np.empty(0), []

        # Enumerate over all bottom keypoints
        for i, bottom in enumerate(bottoms):
            # The possible top keypoint matches must be to the top-right of the bottom
            targets = tops[
                np.logical_and(tops[:, 0] > bottom[0], tops[:, 1] < bottom[1])
            ]

            # If there are none, too bad:(
            if len(targets) == 0:
                continue

            # Find the closest match
            diffs = targets - bottom
            errors = np.sum(np.power(diffs, 2), axis=1)
            best_match = np.argmin(errors)
            lowest_diff = diffs[best_match][0] ** 2 + diffs[best_match][1] ** 2

            # If the closest top is within the threshold, choose it!
            threshold = self.matching_threshold_px**2
            # Large cones get a larger matching threshold
            if category == 2:
                threshold += 50**2
            if lowest_diff >= threshold:
                continue

            # Only detect cones that are large enough
            height = bottom[1] - targets[best_match, 1]
            if height < self.detection_height_threshold:
                continue

            heights = np.append(heights, height)
            valid_bottom_idx.append(i)

        # If we couldn't find any cones, too bad:(
        if len(heights) == 0:
            return np.empty(0)

        # For each matched pair, calculate the cone position
        categories = np.full(heights.shape[0], category, dtype=int)
        cones = self.height_to_pos(categories, heights, bottoms[valid_bottom_idx, :])

        return cones

    def predict(self, original_image: npt.ArrayLike):
        """Given an image, pass it through the global keypoint detector, match the keypoints, and return the cones that were found.

        Args:
            original_image: the numpy image object of (H, W, 3)

        Returns:
            - an array of Nx4 containing (cone category, x, y, z)
            - the pipeline latencies containing triplets of (pre-processing, kpt inference, post-processing)
            - (optional) a visualisation image
        """
        # Don't sync GPU and CPU for timing
        # This is less acurate, but does not slow down the code as much
        latencies = []

        # The image should be on the GPU and 3xHxW
        start = time.perf_counter()
        image = torch.from_numpy(original_image).to(self.device).permute(2, 0, 1)
        latencies.append(1000 * (time.perf_counter() - start))

        # Detect keypoints
        start = time.perf_counter()
        original_size = image.shape[1:]
        if self.image_encoding is not None:
            image = self.image_encoding(
                image
            )  # Don't forget to rescale the image before inference
        image = image.unsqueeze(0).to(self.device).float() / 255
        with torch.inference_mode():
            keypoints = self.model(image)
        latencies.append(1000 * (time.perf_counter() - start))

        # Extract keypoints
        start = time.perf_counter()

        # Use the GPU-accelerated keypoint extractor (re-written by Thomas Lips)
        # The result are the extracted keypoints ((u, v) coordinates) for each batch (1), channel (8)

        # List with elements: Array of #detections x 2
        # There are 8 detection channels:
        #   blue_top, blue_bottom, yellow_top, ...
        kpts = self.get_keypoints_from_heatmap_batch_maxpool(keypoints)[
            0
        ]  # There should only be one batch, take that one

        # The resulting list should be 8 lists of unknown length (possibly empty)
        for i in range(8):
            if len(kpts[i]) == 0:
                kpts[i] = np.empty((0, 2))
                continue

            # Convert the list to a Numpy array
            kpts[i] = np.array(kpts[i])

            if self.image_encoding is not None:
                # Don't forget to undo any image resizing afterwards
                kpts[i][:, 0] = (
                    kpts[i][:, 0] * original_size[1] / self.image_size[1]
                ).astype(int)
                kpts[i][:, 1] = (
                    kpts[i][:, 1] * original_size[0] / self.image_size[0]
                ).astype(int)
        latencies.append(1000 * (time.perf_counter() - start))

        # Find cone locations
        start = time.perf_counter()
        cones = np.empty((0, 4))
        for i in range(4):
            # If there are both top and bottom keypoints, run the processing code
            if kpts[2 * i].shape[0] > 0 and kpts[2 * i + 1].shape[0] > 0:
                extracted_cones = self.extract_cones(kpts[2 * i], kpts[2 * i + 1], i)
                if extracted_cones.size > 0:
                    cones = np.vstack((cones, extracted_cones))
        latencies.append(1000 * (time.perf_counter() - start))

        vis_img = None
        if self.use_vis:
            vis_img = np.copy(original_image)
            vis_img = cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR)
            vis_img = self.visualise(vis_img, kpts)
            vis_img = cv2.cvtColor(vis_img, cv2.COLOR_BGR2RGB)

        return cones, latencies, vis_img

    def get_keypoints_from_heatmap_batch_maxpool(
        self,
        heatmap: torch.Tensor,
        max_keypoints: int = 20,
        min_keypoint_pixel_distance: int = 1,
        abs_max_threshold: Optional[float] = None,
        rel_max_threshold: Optional[float] = None,
        return_scores: bool = False,
    ) -> List[List[List[Tuple[int, int]]]]:
        """Fast extraction of keypoints from a batch of heatmaps using maxpooling.

        Taken from:
        https://github.com/tlpss/keypoint-detection
        Inspired by mmdetection and CenterNet:
        https://mmdetection.readthedocs.io/en/v2.13.0/_modules/mmdet/models/utils/gaussian_target.html

        Args:
            heatmap (torch.Tensor): NxCxHxW heatmap batch
            max_keypoints (int, optional): max number of keypoints to extract, lowering will result in faster execution times. Defaults to 20.
            min_keypoint_pixel_distance (int, optional): _description_. Defaults to 1.

            Following thresholds can be used at inference time to select where you want to be on the AP curve. They should ofc. not be used for training
            abs_max_threshold (Optional[float], optional): _description_. Defaults to None.
            rel_max_threshold (Optional[float], optional): _description_. Defaults to None.

        Returns:
            The extracted keypoints for each batch, channel and heatmap; and their scores
        """

        # TODO: maybe separate the thresholding into another function to make sure it is not used during training, where it should not be used?

        # TODO: ugly that the output can change based on a flag.. should always return scores and discard them when I don't need them...

        batch_size, n_channels, _, width = heatmap.shape

        # obtain max_keypoints local maxima for each channel (w/ maxpool)

        kernel = min_keypoint_pixel_distance * 2 + 1
        pad = min_keypoint_pixel_distance
        # exclude border keypoints by padding with highest possible value
        # bc the borders are more susceptible to noise and could result in false positives
        padded_heatmap = torch.nn.functional.pad(
            heatmap, (pad, pad, pad, pad), mode="constant", value=1.0
        )
        max_pooled_heatmap = torch.nn.functional.max_pool2d(
            padded_heatmap, kernel, stride=1, padding=0
        )
        # if the value equals the original value, it is the local maximum
        local_maxima = max_pooled_heatmap == heatmap
        # all values to zero that are not local maxima
        heatmap = heatmap * local_maxima

        # extract top-k from heatmap (may include non-local maxima if there are less peaks than max_keypoints)
        scores, indices = torch.topk(
            heatmap.view(batch_size, n_channels, -1), max_keypoints, sorted=True
        )
        indices = torch.stack(
            [torch.div(indices, width, rounding_mode="floor"), indices % width], dim=-1
        )
        # at this point either score > 0.0, in which case the index is a local maximum
        # or score is 0.0, in which case topk returned non-maxima, which will be filtered out later.

        #  remove top-k that are not local maxima and threshold (if required)
        # thresholding shouldn't be done during training

        #  moving them to CPU now to avoid multiple GPU-mem accesses!
        indices = indices.detach().cpu().numpy()
        scores = scores.detach().cpu().numpy()
        filtered_indices = [[[] for _ in range(n_channels)] for _ in range(batch_size)]
        filtered_scores = [[[] for _ in range(n_channels)] for _ in range(batch_size)]
        # determine NMS threshold
        threshold = (
            0.01  # make sure it is > 0 to filter out top-k that are not local maxima
        )
        if abs_max_threshold is not None:
            threshold = max(threshold, abs_max_threshold)
        if rel_max_threshold is not None:
            threshold = max(threshold, rel_max_threshold * heatmap.max())

        # have to do this manually as the number of maxima for each channel can be different
        for batch_idx in range(batch_size):
            for channel_idx in range(n_channels):
                candidates = indices[batch_idx, channel_idx]
                for candidate_idx in range(candidates.shape[0]):
                    # these are filtered out directly.
                    if scores[batch_idx, channel_idx, candidate_idx] > threshold:
                        # convert to (u,v)
                        filtered_indices[batch_idx][channel_idx].append(
                            candidates[candidate_idx][::-1].tolist()
                        )
                        filtered_scores[batch_idx][channel_idx].append(
                            scores[batch_idx, channel_idx, candidate_idx]
                        )
        if return_scores:
            return filtered_indices, filtered_scores
        else:
            return filtered_indices

    def visualise(self, image, kpts):
        for i in range(4):
            category = i
            keypoints = np.rint(np.vstack((kpts[2 * i], kpts[2 * i + 1]))).astype(int)
            for pred in keypoints:
                image = cv2.circle(image, pred, 3, CAT_TO_COLOUR[category], -1)
                image = cv2.circle(image, pred, 2, (0, 0, 0), -1)
                image = cv2.circle(image, pred, 1, (255, 255, 255), -1)

        return image
