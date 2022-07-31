from dataclasses import dataclass
from typing import List


@dataclass
class BoundingBox:
    top_left_x: float
    top_left_y: float
    width: float
    height: float
    class_id: int
    confidence: float


@dataclass
class YOLOv5Output:
    """Class for keeping track of detections in image"""

    bounding_boxes: List[BoundingBox]
