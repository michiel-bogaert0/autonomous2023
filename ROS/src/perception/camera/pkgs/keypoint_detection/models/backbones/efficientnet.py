from functools import reduce
from operator import __add__

import timm
import torch
import torch.nn as nn
from keypoint_detection.models.backbones.base_backbone import Backbone
from keypoint_detection.models.backbones.mobilenetv3 import (Conv2dSamePadding,
                                                             UpSamplingBlock)


class EfficientNet(Backbone):
    """
    Pretrained EfficientNetB0
    """

    def __init__(self, **kwargs):
        super().__init__()
        self.encoder = timm.create_model(
            "efficientnet_b0", pretrained=True, features_only=True
        )
        self.decoder_blocks = nn.ModuleList()
        for i in range(1, len(self.encoder.feature_info.info)):
            channels_in, skip_channels_in = (
                self.encoder.feature_info.info[-i]["num_chs"],
                self.encoder.feature_info.info[-i - 1]["num_chs"],
            )
            block = UpSamplingBlock(channels_in, skip_channels_in, skip_channels_in, 3)
            self.decoder_blocks.append(block)

        self.final_upsampling_block = nn.Sequential(
            nn.UpsamplingBilinear2d(scale_factor=2),
            Conv2dSamePadding(skip_channels_in, skip_channels_in, 3),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        features = self.encoder(x)

        x = features.pop()
        for block in self.decoder_blocks:
            x = block(x, features.pop())
        x = self.final_upsampling_block(x)

        return x

    def get_n_channels_out(self):
        return self.encoder.feature_info.info[0]["num_chs"]
