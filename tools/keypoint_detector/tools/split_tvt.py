"""
File written to split the dataset into train/val/test whereby each dataset is located in its own folder and the
labels are written similarly to YOLO whereby the label file has the same filename but json extension.
"""
import json
import random
import shutil
from pathlib import Path

dataset_dir = Path.home() / "datasets" / "rektnet"
imgs = list((dataset_dir / "images").rglob("*.jpg"))

label_file = dataset_dir / "labels.json"
image_folder = dataset_dir / "images"

with open(label_file) as f:
    dataset_labels = json.load(f)

images = [label_seq[0] for label_seq in dataset_labels]
labels = [label_seq[1] for label_seq in dataset_labels]

indices = list(range(len(labels)))
random.seed(42)  # We want the data to be shuffled, but not random -> otherwise metrics are hard to compare
random.shuffle(indices)

val_count = int(0.15 * len(labels))
test_count = int(0.15 * len(labels))
train_count = len(labels) - val_count - test_count

train_indices = indices[:train_count]
val_indices = indices[train_count : train_count + val_count]
test_indices = indices[train_count + val_count :]


assert set(train_indices).isdisjoint(set(val_indices))
assert set(val_indices).isdisjoint(set(test_indices))


def copy_indices(mov_indices, dest):
    for idx in mov_indices:
        img_name = images[idx]
        shutil.copy(image_folder / img_name, dest / img_name)
        label_name = img_name.replace(".jpg", ".json")
        with open(dest / label_name, "w") as f:
            json.dump(labels[idx], f)


copy_indices(train_indices, dataset_dir / "train")
copy_indices(val_indices, dataset_dir / "val")
copy_indices(test_indices, dataset_dir / "test")
