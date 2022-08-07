import sys
from pathlib import Path
from typing import Any, List, Tuple

import numpy as np
import torch
from yolov5.models.common import DetectMultiBackend
from yolov5.output import BoundingBox, YOLOv5Output
from yolov5.utils.datasets import LoadImage
from yolov5.utils.general import (
    check_img_size,
    non_max_suppression,
    scale_coords,
    strip_optimizer,
    xyxy2xywh,
)
from yolov5.utils.torch_utils import select_device

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())  # add yolov5/ to path

DEFAULT_DEVICE = ""


def preprocess_img(img: List, half: bool) -> Any:
    """Make image ready for inference/training
    First load image on GPU if available, otherwise load on CPU
    Normalize color ranges of image
    Perform unsqueezing

    Args:
        img: Image that needs preprocessing before inference/training -> List
        half: Boolean, if true only half precision will be used (only available for CUDA) -> bool

    Returns:
        preprocessed image, ready for further processing -> List
    """
    device = select_device(DEFAULT_DEVICE)

    # Move image to right compute unit
    img_torch = torch.from_numpy(img).to(device)

    # uint8 to fp16/32
    img_torch = img_torch.half() if half else img_torch.float()

    # Normalize color ranges
    img_torch /= 255.0

    # Unsqueeze image
    if img_torch.ndimension() == 3:
        img_torch = img_torch.unsqueeze(0)

    return img_torch


def xywh2tlwh(xywh: List) -> List:
    """
    Translate the x and y coordinates of the center of the bounding box to the x and y coordinates of top left point
    xywh: List of values: [x coordinate of center, y coordinate of center, width, heigth]
    xywh: List
    :return: [x coordinate of top left, y coordinate of top left, width, heigth]
    :rtype: List
    """
    x, y, w, h = xywh[0], xywh[1], xywh[2], xywh[3]
    return [x - w / 2, y - h / 2, w, h]


def process_detections(
    pred: List, original_image: np.ndarray, img: np.ndarray
) -> YOLOv5Output:
    """
    Process detections made in a specific image
    pred: Predictions in the specific image
    pred: List
    original_image: original input image
    original_image: np.ndarray
    img: image after rescaling and preprocessing
    img: np.ndarray
    :return: Output bounding boxes
    :rtype: YOLOv5Output
    """
    output = YOLOv5Output(bounding_boxes=[])
    for i, det in enumerate(pred):
        original_image_copy = original_image.copy()

        # Normalization gain whwh
        gn = torch.tensor(original_image_copy.shape)[[1, 0, 1, 0]]

        if len(det):
            # Rescale boxes from img_size to original_image_copy size
            det[:, :4] = scale_coords(
                img.shape[2:], det[:, :4], original_image_copy.shape
            ).round()

        for *xyxy, conf, cls in reversed(det):
            # Normalized xywh
            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
            tlwh = xywh2tlwh(xywh)

            bounding_box = BoundingBox(
                top_left_x=tlwh[0],
                top_left_y=tlwh[1],
                width=tlwh[2],
                height=tlwh[3],
                class_id=int(cls),
                confidence=conf,
            )
            output.bounding_boxes.append(bounding_box)

    return output


class YOLOv5:
    def __init__(
        self,
        weight_file: Path,
        data: Path,
        image_size: Tuple[int] = (640, 640),
        conf_thres: float = 0.25,
        iou_thres: float = 0.45,
        max_det: int = 1000,
        device: str = "",
        classes: List = None,
        agnostic_nms: bool = False,
        augment: bool = False,
        update: bool = False,
        half: bool = False,
        dnn: bool = False,
    ) -> None:
        """
        Initialize YOLOv5 class

        Args:
            weight_file: Path to the weigths file -> Path
            data: Path to the yaml dataset file -> Path
            image_size: Size of image for inference -> int
            conf_thres: Confidence threshold for detections -> float
            iou_thres: Intersection over Union threshold -> float
            max_det: Max amount of detections per image -> int
            device: Compute device used -> str
            classes: Classes used for infernce and training -> List
            agnostic_nms: Use agnostic non max supression -> bool
            augment: Augment image for inference -> bool
            update: Update model to fix SourceChangeWarning -> bool
            half: Use half precision -> bool
        """
        self.weight_file = weight_file
        self.image_size = image_size
        self.data = data
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.device = device
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.augment = augment
        self.update = update
        self.dnn = dnn
        self.half = half
        self.model = None
        self.names: List = []
        self.stride: int = 0

        self.load_model(str(self.weight_file))

    def load_model(self, weights: str) -> Tuple[Any, List, int]:
        """
        First attempt to load weights, if weights not availaible -> automatically download
        them. Then load weights and store in model. Also checks whether image size is a multiple of stride

        Args:
            weights: name of file of weights used for inference/training -> str

        Returns:
            loaded model, all class names in list and stride -> Tuple[Any, List, int]
        """
        self.device = select_device(self.device)

        # Load model
        model = DetectMultiBackend(
            weights, device=self.device, dnn=self.dnn, data=self.data, fp16=self.half
        )
        stride, names, pt = model.stride, model.names, model.pt

        self.image_size = check_img_size(self.image_size, s=stride)
        if self.half:
            model.half()  # to FP16

        self.model = model
        self.names = names
        self.pt = pt
        self.stride = stride

    def infer(self, image: np.ndarray) -> YOLOv5Output:
        """
        Perform inference for a given input image
        image: input image for inference
        image: np.ndarray
        :return: Bounding boxes for detections in input image
        :rtype: YOLOv5Output
        """
        assert self.model is not None, f"Model is not loaded correctly"

        # Half precision only supported on CUDA
        self.half &= self.device.type != "cpu"

        # Load dataset
        dataset = LoadImage(image=image, img_size=self.image_size, stride=self.stride)

        # Get image and preprocess
        img, original_image = dataset.get_image()
        img = preprocess_img(img, self.half)

        # Inference
        self.model.warmup(imgsz=(1, 3, *self.image_size))  # warmup
        pred = self.model(img, augment=self.augment)

        # Apply NMS
        pred = non_max_suppression(
            pred,
            self.conf_thres,
            self.iou_thres,
            self.classes,
            self.agnostic_nms,
            max_det=self.max_det,
        )

        # Process detections
        bounding_boxes = process_detections(pred, original_image, img)

        # Update model (to fix SourceChangeWarning)
        if self.update:
            strip_optimizer(self.weights)

        return bounding_boxes
