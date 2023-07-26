import cv2
import numpy as np
import numpy.typing as npt
import torch
import time
import cv2
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

        # List with elements: Array of #detections x 2
        kpts = []
        # There are 8 detection channels:
        #   blue_top, blue_bottom, yellow_top, ...
        for i in range(8):
            # Use the GPU-accelerated keypoint extractor
            arr = self.extract_keypoints_from_heatmap(keypoints[0, i])

            if len(arr) == 0:
                arr = np.empty((0, 2))
            elif self.image_encoding is not None:
                # Don't forget to undo any image resizing afterwards
                arr[:, 0] *= original_size[1] / self.image_size[1]
                arr[:, 1] *= original_size[0] / self.image_size[0]
            kpts.append(arr)
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

    def extract_keypoints_from_heatmap(self, heatmap, nms_range = 15, threshold = 0.1):
        """Extracts keypoitns from a heatmap using GPU acceleration
        
        Args:
            heatmap: (H, W) array
            nms_range: the range for NMS in pixels
            threshold: the minimal heatmap value to be considered a keypoint
        
        Returns:
            array of Nx2 containing (u, v) coordinates
        """

        output_points = torch.empty((0, 2), dtype=float, device=self.device)
        nms_range_squared = nms_range ** 2

        kpt_candidates_x, kpt_candidates_y = torch.where(heatmap > threshold)
        kpt_candidates = torch.vstack((kpt_candidates_x, kpt_candidates_y)).T
        kpt_candidates_scores = heatmap[heatmap > threshold]
        order = torch.argsort(kpt_candidates_scores, descending=True)
        kpt_candidates = kpt_candidates[order]

        # From lowest to highest score
        # Only add the points without a higher point in the NMS range
        while kpt_candidates.shape[0] > 0:
            kpt = kpt_candidates[0]
            output_points = torch.vstack((output_points, kpt))
            distances = torch.pow(kpt_candidates - kpt, 2).sum(dim=1)

            # Identify points outside of NMS range and keep them
            keypoints_outside_nms_mask = distances > nms_range_squared
            kpt_candidates = kpt_candidates[keypoints_outside_nms_mask]

        return output_points.cpu().numpy()[..., ::-1]

    def visualise(self, image, kpts):
        for i in range(4):
            category = i
            keypoints = np.rint(np.vstack((kpts[2 * i], kpts[2 * i + 1]))).astype(int)
            for pred in keypoints:
                image = cv2.circle(image, pred, 3, CAT_TO_COLOUR[category], -1)
                image = cv2.circle(image, pred, 2, (0, 0, 0), -1)
                image = cv2.circle(image, pred, 1, (255, 255, 255), -1)

        return image
