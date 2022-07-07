#! /usr/bin/python3
import os
from pathlib import Path

import numpy as np
from fs_msgs.msg import Cone
from sensor_msgs.msg import Image
from ugr_msgs.msg import BoundingBox
from yolov5 import YOLOv5
from yolov5.output import BoundingBox as YOLOVBoundingBox


class ConeDetector:
    def __init__(self, device: str):
        model_path = Path(os.getenv("BINARY_LOCATION")) / "nn_models" / "yolov5.engine"
        data_path = (
            Path(os.getenv("BINARY_LOCATION")) / "perception_data" / "ugr_dataset.yaml"
        )
        self.yolo_model = YOLOv5(model_path, data_path, device=device)

    def yolo_bb_to_ros_bb(self, input_bb: "YOLOVBoundingBox") -> "BoundingBox":
        """
        Converts yolo style bounding boxes to ROS bbs

        Args:
            input_bb: YOLO bounding box

        Returns:
            BoundingBox
        """
        if self.yolo_model.names[input_bb.class_id] == "blue_cone":
            cone_type = Cone.BLUE
        if self.yolo_model.names[input_bb.class_id] == "yellow_cone":
            cone_type = Cone.YELLOW
        if self.yolo_model.names[input_bb.class_id] == "orange_cone":
            cone_type = Cone.ORANGE_SMALL

        return BoundingBox(
            cone_type=cone_type,
            score=input_bb.confidence,
            left=input_bb.top_left_x,
            top=input_bb.top_left_y,
            width=input_bb.width,
            height=input_bb.height,
        )

    def detect_cones(self, image: np.ndarray) -> list("BoundingBox"):
        """
        Run YOLOv5 inference on an image
        Args:
            image: The input image

        Returns:
        The bounding boxes of the detected cones as BoundingBox
        """
        output = self.yolo_model.infer(image)

        ros_bbs = [self.yolo_bb_to_ros_bb(bb) for bb in output.bounding_boxes]

        return ros_bbs
