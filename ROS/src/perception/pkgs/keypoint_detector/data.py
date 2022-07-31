import json
import math
import random
from pathlib import Path
from typing import Optional, Tuple

import albumentations as A
import numpy as np
import torch
from keypoint_detector.param import RektNetTrainParam
from PIL import Image
from pytorch_lightning import LightningDataModule
from scipy.stats import multivariate_normal
from torch.utils.data import DataLoader, Dataset
from torchvision.transforms.functional import to_tensor


def create_single_heatmap(x, y, img_size, sigma=2):
    pos = np.dstack(np.mgrid[0 : img_size[1] : 1, 0 : img_size[0] : 1])
    rv = multivariate_normal(mean=[y, x], cov=sigma)
    heatmap = rv.pdf(pos)
    return heatmap / heatmap.sum()


def create_heatmaps(labels, img_size):
    heatmaps = []
    for hm_idx, (x, y) in enumerate(labels):
        heatmaps.append(create_single_heatmap(x, y, img_size))
    return np.stack(heatmaps)


def retrieve_label_file(location: Path, img_size: Tuple[int, int]):
    with open(location) as f:
        loaded_labels = np.array(json.load(f))

    loaded_labels[:, 0] *= img_size[0]
    loaded_labels[:, 1] *= img_size[1]

    return loaded_labels


def retrieve_data(location: Path, img_size):
    images = list(location.rglob("*.jpg"))
    labels = []
    for img in images:
        label_file = img.parent / img.parts[-1].replace(".jpg", ".json")
        labels.append(retrieve_label_file(label_file, img_size))

    return images, labels


def read_img(img_path: Path, img_size):
    img = Image.open(img_path)
    return np.array(img.resize(img_size))


def flip_img(img, label):
    img = np.fliplr(img)
    flip_pairs = [(1, 2), (3, 4), (5, 6)]
    new_label = label.copy()

    for idx1, idx2 in flip_pairs:
        new_label[idx1], new_label[idx2] = new_label[idx2], new_label[idx1]

    return img, new_label


class ConeDataset(Dataset):
    def __init__(
        self, location: Path, target_image_size, transform=None, apply_augmentation=True
    ):
        self.images, self.labels = retrieve_data(location, target_image_size)
        self.target_image_size = target_image_size
        self.transform = transform

        self.apply_augmentation = apply_augmentation

        self.aug_steps = A.Compose(
            [
                A.SafeRotate(limit=15, p=1),
                A.RandomBrightnessContrast(p=1),
                A.RGBShift(
                    r_shift_limit=100, g_shift_limit=100, b_shift_limit=100, p=1
                ),
                A.Blur(p=0.1, blur_limit=4),
            ],
            keypoint_params=A.KeypointParams(format="xy"),
        )

    def __len__(self):
        return len(self.images)

    def get_python_objects(self, index: int) -> Tuple[Image.Image, np.array]:
        img = read_img(self.images[index], self.target_image_size)
        label = self.labels[index]

        if self.apply_augmentation:
            img, label = self.augment(img, label)

        heatmaps = create_heatmaps(label, self.target_image_size)

        return img.copy(), heatmaps

    def augment(self, img, label):
        res = self.aug_steps(image=img, keypoints=label)
        n_img, n_label = res["image"], res["keypoints"]

        if len(n_label) != len(label):
            # Sometimes the augmentation removes keypoints due to rotation / scaling
            # Influence should not be to big
            n_img, n_label = self.augment(img, label)

        # if random.random() > 0.5:
        #     n_img, n_label = flip_img(n_img, n_label)

        return n_img, n_label

    def __getitem__(self, index: int):
        img, heatmaps = self.get_python_objects(index)

        return (
            to_tensor(img),
            # torch.from_numpy(img.copy()).type("torch.FloatTensor"),
            torch.from_numpy(heatmaps).type("torch.FloatTensor"),
            torch.from_numpy(np.array(self.labels[index])).type("torch.FloatTensor"),
        )


class KeyPointDataset(LightningDataModule):
    def __init__(
        self,
        hyperparam,
        train_param: RektNetTrainParam,
        data_folder: Path = Path.cwd() / "dataset",
    ):
        super().__init__()
        self.data_folder = data_folder
        self.label_file = self.data_folder / "labels.json"
        self.image_folder = self.data_folder / "images"

        self.keypoint_keys = [
            "top",
            "mid_L_top",
            "mid_R_top",
            "mid_L_bot",
            "mid_R_bot",
            "bot_L",
            "bot_R",
        ]

        self.hyperparam = hyperparam
        self.train_param = train_param

        self.num_workers = 12

        self.train_folder = self.data_folder / "train"
        self.val_folder = self.data_folder / "val"
        self.test_folder = self.data_folder / "test"

        self.img_size = (self.hyperparam.img_width, self.hyperparam.img_height)

    def prepare_data(self):
        # called only on 1 GPU
        pass

    def train_dataloader(self):
        return DataLoader(
            ConeDataset(
                self.train_folder,
                self.img_size,
                apply_augmentation=self.hyperparam.apply_augmentation,
            ),
            batch_size=self.hyperparam.batch_size,
            num_workers=self.num_workers,
        )

    def val_dataloader(self):
        return DataLoader(
            ConeDataset(self.val_folder, self.img_size),
            batch_size=self.hyperparam.batch_size,
            num_workers=self.num_workers,
        )

    def test_dataloader(self):
        return DataLoader(
            ConeDataset(self.test_folder, self.img_size),
            batch_size=self.hyperparam.batch_size,
            num_workers=self.num_workers,
        )
