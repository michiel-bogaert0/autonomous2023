#! /usr/bin/python3
from dataclasses import dataclass
from typing import Tuple

import numpy as np
from ugr_msgs.msg import BoundingBox


@dataclass
class IntBoundingBox:
    """
    Defines a Bounding Box for cone detection
    """

    left: int
    top: int
    width: int
    height: int
    right: int
    bottom: int

    tl: Tuple[int, int] = (0, 0)
    br: Tuple[int, int] = (0, 0)

    def __post_init__(self):
        self.tl = (self.left, self.top)
        self.br = (self.right, self.bottom)

    @staticmethod
    def from_img(or_bb: BoundingBox, img: np.ndarray):
        h, w, c = img.shape
        bb_left = int(w * or_bb.left)
        bb_top = int(h * or_bb.top)
        bb_w = int(w * or_bb.width)
        bb_h = int(h * or_bb.height)
        bb_right = bb_left + bb_w
        bb_bottom = bb_top + bb_h

        return IntBoundingBox(
            left=bb_left,
            top=bb_top,
            width=bb_w,
            height=bb_h,
            right=bb_right,
            bottom=bb_bottom,
        )
