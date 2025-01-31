import argparse
from functools import reduce
from operator import __add__
from typing import Any, Dict, List

import pytorch_lightning as pl
import torch
import torch.nn as nn
import wandb
from keypoint_detection.models.backbones.base_backbone import Backbone
from keypoint_detection.models.metrics import (
    DetectedKeypoint,
    Keypoint,
    KeypointAPMetrics,
)
from keypoint_detection.utils.heatmap import (
    BCE_loss,
    compute_keypoint_probability,
    create_heatmap_batch,
    get_keypoints_from_heatmap,
)
from keypoint_detection.utils.visualization import visualize_predictions
from torch.optim.lr_scheduler import ReduceLROnPlateau


class Conv2dSamePadding(nn.Conv2d):
    def __init__(self, *args, **kwargs):
        super(Conv2dSamePadding, self).__init__(*args, **kwargs)
        self.zero_pad_2d = nn.ZeroPad2d(
            reduce(
                __add__,
                [
                    (k // 2 + (k - 2 * (k // 2)) - 1, k // 2)
                    for k in self.kernel_size[::-1]
                ],
            )
        )

    def forward(self, input):
        return self._conv_forward(self.zero_pad_2d(input), self.weight, self.bias)


class KeypointDetector(pl.LightningModule):
    """
    keypoint Detector using Spatial Heatmaps.
    There can be N channels of keypoints, each with its own set of ground truth keypoints.
    The mean Average precision is used to calculate the performance.

    """

    @staticmethod
    def add_model_argparse_args(
        parent_parser: argparse.ArgumentParser,
    ) -> argparse.ArgumentParser:
        """
        add named arguments from the init function to the parser
        """
        parser = parent_parser.add_argument_group("KeypointDetector")

        parser.add_argument(
            "--heatmap_sigma",
            default=2,
            type=int,
            help="The size of the Gaussian blobs that are used to create the ground truth heatmaps from the keypoint coordinates.",
        )
        parser.add_argument(
            "--minimal_keypoint_extraction_pixel_distance",
            type=int,
            default=1,
            help="the minimal pixel-distance between two keypoints. Allows for some non-maximum surpression.",
        )
        parser.add_argument(
            "--maximal_gt_keypoint_pixel_distances",
            type=str,
            default="2 4",
            help="The treshold distance(s) for the AP metric to consider a detection as a True Positive. Separate multiple values by a space to compute the AP for all values.",
        )
        parser.add_argument(
            "--learning_rate", type=float, default=3e-4
        )  # Karpathy constant
        parser.add_argument(
            "--ap_epoch_start",
            type=int,
            default=1,
            help="Epoch at which to start calculating the AP every `ap_epoch_frequency` epochs.",
        )
        parser.add_argument(
            "--ap_epoch_freq",
            type=int,
            default=2,
            help="Rate at which to calculate the AP metric if epoch > `ap_epoch_start`",
        )
        parser.add_argument(
            "--lr_scheduler_relative_threshold",
            default=0.0,
            type=float,
            help="relative threshold for the OnPlateauLRScheduler. If the training epoch loss does not decrease with this fraction for 2 consective epochs, lr is decreased with factor 10.",
        )
        parser.add_argument(
            "--max_keypoints",
            default=20,
            type=int,
            help="the maximum number of keypoints to predict from the generated heatmaps. If set to -1, skimage will look for all peaks in the heatmap, if set to N (N>0) it will return the N most most certain ones.",
        )
        return parent_parser

    def __init__(
        self,
        heatmap_sigma: int,
        maximal_gt_keypoint_pixel_distances: str,
        minimal_keypoint_extraction_pixel_distance: int,
        learning_rate: float,
        backbone: Backbone,
        keypoint_channel_configuration: List[List[str]],
        ap_epoch_start: int,
        ap_epoch_freq: int,
        lr_scheduler_relative_threshold: float,
        max_keypoints: int,
        **kwargs,
    ):
        """[summary]

        Args:
            see argparse help strings for documentation.

            kwargs: Pythonic catch for the other named arguments, used so that we can use a dict with ALL system hyperparameters to initialise the model from this
                    hyperparamater configuration dict. The alternative is to add a single 'hparams' argument to the init function, but this is imo less readable.
                    cf https://pytorch-lightning.readthedocs.io/en/stable/common/hyperparameters.html for an overview.
        """
        super().__init__()
        # No need to manage devices ourselves, pytorch.lightning does all of that.
        # device can be accessed through self.device if required.

        # to add new hyperparameters:
        # 1. define as named arg in the init (and use them)
        # 2. add to the argparse method of this module
        # 3. pass them along when calling the train.py file to override their default value

        self.learning_rate = learning_rate
        self.heatmap_sigma = heatmap_sigma
        self.ap_epoch_start = ap_epoch_start
        self.ap_epoch_freq = ap_epoch_freq
        self.minimal_keypoint_pixel_distance = (
            minimal_keypoint_extraction_pixel_distance
        )
        self.lr_scheduler_relative_threshold = lr_scheduler_relative_threshold
        self.max_keypoints = max_keypoints
        self.keypoint_channel_configuration = keypoint_channel_configuration
        # parse the gt pixel distances
        if isinstance(maximal_gt_keypoint_pixel_distances, str):
            maximal_gt_keypoint_pixel_distances = [
                float(val)
                for val in maximal_gt_keypoint_pixel_distances.strip().split(" ")
            ]
        self.maximal_gt_keypoint_pixel_distances = maximal_gt_keypoint_pixel_distances

        self.ap_validation_metrics = [
            KeypointAPMetrics(self.maximal_gt_keypoint_pixel_distances)
            for _ in self.keypoint_channel_configuration
        ]

        self.ap_test_metrics = [
            KeypointAPMetrics(self.maximal_gt_keypoint_pixel_distances)
            for _ in self.keypoint_channel_configuration
        ]

        self.n_heatmaps = len(self.keypoint_channel_configuration)

        head = Conv2dSamePadding(
            in_channels=backbone.get_n_channels_out(),
            out_channels=self.n_heatmaps,
            kernel_size=(3, 3),
        )

        # expect output of backbone to be normalized!
        # so by filling bias to -4, the sigmoid should be on avg sigmoid(-4) =  0.02
        # which is consistent with the desired heatmaps that are zero almost everywhere.
        # setting too low would result in loss of gradients..
        head.bias.data.fill_(-4)

        self.unnormalized_model = nn.Sequential(
            backbone,
            head,
        )  # NO sigmoid to combine it in the loss! (needed for FP16)

        # save hyperparameters to logger, to make sure the model hparams are saved even if
        # they are not included in the config (i.e. if they are kept at the defaults).
        # this is for later reference (e.g. checkpoint loading) and consistency.
        self.save_hyperparameters(ignore=["**kwargs", "backbone"])

    def forward(self, x: torch.Tensor):
        """
        x shape must be of shape (N,3,H,W)
        returns tensor with shape (N, n_heatmaps, H,W)
        """
        return torch.sigmoid(self.forward_unnormalized(x))

    def forward_unnormalized(self, x: torch.Tensor):
        return self.unnormalized_model(x)

    def configure_optimizers(self):
        """
        Configures an Adam optimizer with ReduceLROnPlateau scheduler. To disable the scheduler, set the relative threshold < 0.
        """
        # YOLOv8
        # self.optimizer = torch.optim.Adam(self.parameters(), lr=self.learning_rate, betas=(0.937, 0.999))

        # Normal
        self.optimizer = torch.optim.Adam(self.parameters(), self.learning_rate)
        self.lr_scheduler = ReduceLROnPlateau(
            self.optimizer,
            threshold=self.lr_scheduler_relative_threshold,
            threshold_mode="rel",
            mode="min",
            factor=0.1,
            patience=2,
            verbose=True,
        )
        return {
            "optimizer": self.optimizer,
            "lr_scheduler": {
                "scheduler": self.lr_scheduler,
                "monitor": "train/loss_epoch",
                "frequency": 1
                # If "monitor" references validation metrics, then "frequency" should be set to a
                # multiple of "trainer.check_val_every_n_epoch".
            },
        }

    def shared_step(
        self, batch, batch_idx, include_visualization_data_in_result_dict=False
    ) -> Dict[str, Any]:
        """
        shared step for train and validation step that computes the heatmaps and losses and
        creates a result dict for later use in the train, validate and test step.

        batch: img, keypoints
        where img is a Nx3xHxW tensor
        and keypoints a nested list of len(channels) x N with K_ij x 2 tensors containing the keypoints for each channel and each sample in the batch

        returns:

        shared_dict (Dict): a dict with a.o. heatmaps, gt_keypoints and losses
        """
        input_images, keypoint_channels = batch

        heatmap_shape = input_images[0].shape[1:]

        gt_heatmaps = [
            create_heatmap_batch(
                heatmap_shape, keypoint_channel, self.heatmap_sigma, self.device
            )
            for keypoint_channel in keypoint_channels
        ]

        input_images = input_images.to(self.device)

        # predict and compute losses
        predicted_unnormalized_maps = self.forward_unnormalized(input_images)
        predicted_heatmaps = torch.sigmoid(predicted_unnormalized_maps)
        channel_losses = []
        channel_gt_losses = []

        result_dict = {}
        for channel_idx in range(len(self.keypoint_channel_configuration)):
            channel_losses.append(
                # combines sigmoid with BCE for increased stability.
                nn.functional.binary_cross_entropy_with_logits(
                    predicted_unnormalized_maps[:, channel_idx, :, :],
                    gt_heatmaps[channel_idx],
                )
            )
            with torch.no_grad():
                channel_gt_losses.append(
                    BCE_loss(gt_heatmaps[channel_idx], gt_heatmaps[channel_idx])
                )

            # pass losses and other info to result dict
            result_dict.update(
                {
                    f"{self.keypoint_channel_configuration[channel_idx]}_loss": channel_losses[
                        channel_idx
                    ].detach()
                }
            )

        loss = sum(channel_losses)
        gt_loss = sum(channel_gt_losses)
        result_dict.update({"loss": loss, "gt_loss": gt_loss})

        if include_visualization_data_in_result_dict:
            result_dict.update(
                {
                    "input_images": input_images.detach().cpu(),
                    "gt_keypoints": keypoint_channels,
                    "predicted_heatmaps": predicted_heatmaps.detach().cpu(),
                    "gt_heatmaps": gt_heatmaps,
                }
            )

        return result_dict

    def training_step(self, train_batch, batch_idx):
        log_images = batch_idx == 0 and self.current_epoch > 0
        result_dict = self.shared_step(
            train_batch, batch_idx, include_visualization_data_in_result_dict=log_images
        )

        if log_images:
            image_grids = self.visualize_predictions_channels(result_dict)
            self.log_image_grids(image_grids, mode="train")

        for channel_name in self.keypoint_channel_configuration:
            self.log(f"train/{channel_name}", result_dict[f"{channel_name}_loss"])

        # self.log("train/loss", result_dict["loss"])
        self.log("train/gt_loss", result_dict["gt_loss"])
        self.log("train/loss", result_dict["loss"], on_epoch=True)  # also logs steps?
        return result_dict

    def update_ap_metrics(self, result_dict, ap_metrics):
        predicted_heatmaps = result_dict["predicted_heatmaps"]
        gt_keypoints = result_dict["gt_keypoints"]
        for channel_idx in range(len(self.keypoint_channel_configuration)):
            predicted_heatmaps_channel = predicted_heatmaps[:, channel_idx, :, :]
            gt_keypoints_channel = gt_keypoints[channel_idx]
            self.update_channel_ap_metrics(
                predicted_heatmaps_channel,
                gt_keypoints_channel,
                ap_metrics[channel_idx],
            )

    def visualize_predictions_channels(self, result_dict):
        input_images = result_dict["input_images"]
        gt_heatmaps = result_dict["gt_heatmaps"]
        predicted_heatmaps = result_dict["predicted_heatmaps"]

        image_grids = []
        for channel_idx in range(len(self.keypoint_channel_configuration)):
            grid = visualize_predictions(
                input_images,
                predicted_heatmaps[:, channel_idx, :, :],
                gt_heatmaps[channel_idx].cpu(),
                minimal_keypoint_pixel_distance=6,
            )
            image_grids.append(grid)
        return image_grids

    @staticmethod
    def logging_label(channel_configuration, mode: str) -> str:
        channel_name = channel_configuration

        if isinstance(channel_configuration, list):
            if len(channel_configuration) == 1:
                channel_name = channel_configuration[0]
            else:
                channel_name = (
                    f"{channel_configuration[0]}+{channel_configuration[1]}+..."
                )

        channel_name_short = (
            (channel_name[:40] + "...") if len(channel_name) > 40 else channel_name
        )
        label = f"{channel_name_short}_{mode}_keypoints"
        return label

    def log_image_grids(self, image_grids, mode: str):
        for channel_configuration, grid in zip(
            self.keypoint_channel_configuration, image_grids
        ):
            label = KeypointDetector.logging_label(channel_configuration, mode)
            image_caption = "top: predicted heatmaps, middle: predicted keypoints, bottom: gt heatmap"
            self.logger.experiment.log(
                {label: wandb.Image(grid, caption=image_caption)}
            )

    def validation_step(self, val_batch, batch_idx):
        # no need to switch model to eval mode, this is handled by pytorch lightning
        result_dict = self.shared_step(
            val_batch, batch_idx, include_visualization_data_in_result_dict=True
        )

        if self.is_ap_epoch():
            self.update_ap_metrics(result_dict, self.ap_validation_metrics)

        log_images = batch_idx == 0 and self.current_epoch > 0
        if log_images:
            image_grids = self.visualize_predictions_channels(result_dict)
            self.log_image_grids(image_grids, mode="validation")

        # log (defaults to on_epoch, which aggregates the logged values over entire validation set)
        self.log("validation/epoch_loss", result_dict["loss"])
        self.log("validation/gt_loss", result_dict["gt_loss"])

    def test_step(self, test_batch, batch_idx):
        # no need to switch model to eval mode, this is handled by pytorch lightning
        result_dict = self.shared_step(
            test_batch, batch_idx, include_visualization_data_in_result_dict=True
        )
        self.update_ap_metrics(result_dict, self.ap_test_metrics)
        image_grids = self.visualize_predictions_channels(result_dict)
        self.log_image_grids(image_grids, mode="test")
        self.log("test/epoch_loss", result_dict["loss"])
        self.log("test/gt_loss", result_dict["gt_loss"])

    def log_and_reset_mean_ap(self, mode: str):
        mean_ap = 0.0
        metrics = self.ap_test_metrics if mode == "test" else self.ap_validation_metrics

        for channel_idx, channel_name in enumerate(self.keypoint_channel_configuration):
            channel_mean_ap = self.compute_and_log_metrics_for_channel(
                metrics[channel_idx], channel_name, mode
            )
            mean_ap += channel_mean_ap
        mean_ap /= len(self.keypoint_channel_configuration)
        self.log(f"{mode}/meanAP", mean_ap)

    def validation_epoch_end(self, outputs):
        """
        Called on the end of a validation epoch.
        Used to compute and log the AP metrics.
        """
        if self.is_ap_epoch():
            self.log_and_reset_mean_ap("validation")

    def test_epoch_end(self, outputs):
        """
        Called on the end of a test epoch.
        Used to compute and log the AP metrics.
        """
        self.log_and_reset_mean_ap("test")

    def update_channel_ap_metrics(
        self,
        predicted_heatmaps: torch.Tensor,
        gt_keypoints: List[torch.Tensor],
        validation_metric: KeypointAPMetrics,
    ):
        """
        Updates the AP metric for a batch of heatmaps and keypoins of a single channel.
        This is done by extracting the detected keypoints for each heatmap and combining them with the gt keypoints for the same frame, so that
        the confusion matrix can be determined together with the distance thresholds.

        predicted_heatmaps: N x H x W tensor
        gt_keypoints: List of size N, containing K_i x 2 tensors with the ground truth keypoints for the channel of that sample
        """

        # log corner keypoints to AP metrics, frame by frame
        formatted_gt_keypoints = [
            [Keypoint(int(k[0]), int(k[1])) for k in frame_gt_keypoints]
            for frame_gt_keypoints in gt_keypoints
        ]
        for i, predicted_heatmap in enumerate(torch.unbind(predicted_heatmaps, 0)):
            detected_keypoints = self.extract_detected_keypoints_from_heatmap(
                predicted_heatmap
            )
            validation_metric.update(detected_keypoints, formatted_gt_keypoints[i])

    def compute_and_log_metrics_for_channel(
        self, metrics: KeypointAPMetrics, channel: str, training_mode: str
    ) -> float:
        """
        logs AP of predictions of single Channel² for each threshold distance (as configured) for the categorization of the keypoints into a confusion matrix.
        Also resets metric and returns resulting meanAP over all channels.
        """
        # compute ap's
        ap_metrics = metrics.compute()
        print(f"{ap_metrics=}")
        for maximal_distance, ap in ap_metrics.items():
            self.log(f"{training_mode}/{channel}_ap/d={maximal_distance}", ap)

        mean_ap = sum(ap_metrics.values()) / len(ap_metrics.values())

        self.log(
            f"{training_mode}/{channel}_meanAP", mean_ap
        )  # log top level for wandb hyperparam chart.
        metrics.reset()
        return mean_ap

    def is_ap_epoch(self) -> bool:
        """Returns True if the AP should be calculated in this epoch."""
        return (
            self.ap_epoch_start <= self.current_epoch
            and self.current_epoch % self.ap_epoch_freq == 0
        ) or self.current_epoch == self.trainer.max_epochs - 1

    def extract_detected_keypoints_from_heatmap(
        self, heatmap: torch.Tensor
    ) -> List[DetectedKeypoint]:
        """
        Extract keypoints from a single channel prediction and format them for AP calculation.

        Args:
        heatmap (torch.Tensor) : H x W tensor that represents a heatmap.
        """

        detected_keypoints = get_keypoints_from_heatmap(
            heatmap, self.minimal_keypoint_pixel_distance, self.max_keypoints
        )
        keypoint_probabilities = compute_keypoint_probability(
            heatmap, detected_keypoints
        )
        detected_keypoints = [
            DetectedKeypoint(
                detected_keypoints[i][0],
                detected_keypoints[i][1],
                keypoint_probabilities[i],
            )
            for i in range(len(detected_keypoints))
        ]

        return detected_keypoints
