from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from keypoint_detector.data import (ConeDataset, KeyPointDataset,
                                    create_heatmaps, read_img, retrieve_data,
                                    retrieve_label_file)
from keypoint_detector.module import RektNetModule
from param import RektNetTrainParam

dataset_dir = Path.home() / "datasets" / "rektnet" / "train"
images, labels = retrieve_data(dataset_dir, (60, 80))
img, label = np.array(read_img(images[0], (60, 80))), labels[0]


def plotted_img(plt_img, plt_label, color=(0, 255, 0), thickness=1):
    for (px, py), (nx, ny) in zip(plt_label[:-1], plt_label[1:]):
        plt_img = cv2.arrowedLine(
            plt_img, (int(px), int(py)), (int(nx), int(ny)), color, thickness
        )

    return plt_img


img_index = 0


def save_test_sample(img, pt, intered_pt, color=(0, 255, 0), thickness=1):
    global img_index  # Ow yes!
    img_index += 1

    img = (np.moveaxis(img, 0, -1) * 255).astype(np.uint8)
    img = img.copy()  # This is need,
    for (px, py), (nx, ny) in zip(pt, intered_pt):
        img = cv2.arrowedLine(
            img, (int(px), int(py)), (int(nx), int(ny)), color, thickness
        )

    save_loc = Path.cwd() / "test_infer" / f"{str(img_index).zfill(4)}.jpg"

    cv2.imwrite(str(save_loc), img)


def save_test_batch(imgs, pts, infered_pts):
    for img, pt, infered_pt in zip(imgs, pts, infered_pts):
        save_test_sample(img, pt, infered_pt)


checkpoint_path = Path.cwd() / "checkpoints"
model_name = "checkpoint_val_loss=36.73_epoch=199.ckpt"
model_file = str(checkpoint_path / model_name)

model = RektNetModule.load_from_checkpoint(model_file)

dataset_dir = Path.home() / "datasets" / "rektnet"
imgs = list((dataset_dir / "images").rglob("*.jpg"))

data = KeyPointDataset(model.hparams, RektNetTrainParam(), data_folder=dataset_dir)
test_dl = data.test_dataloader()

for imgs, hms, pts in test_dl:
    inf_hms, infered_pts = model(imgs)
    save_test_batch(
        imgs.detach().cpu().numpy(),
        pts.detach().cpu().numpy(),
        infered_pts.detach().cpu().numpy(),
    )
