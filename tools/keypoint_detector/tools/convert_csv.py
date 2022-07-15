"""
File written to convert the original .csv label file to a json label file.
"""


import json
from pathlib import Path

import cv2
import pandas as pd

data_folder = Path.cwd() / "dataset"
csv_file = data_folder / "labels.csv"
img_folder = data_folder / "images"

keypoint_keys = ["top", "mid_L_top", "mid_R_top", "mid_L_bot", "mid_R_bot", "bot_L", "bot_R"]

dataset_table = pd.read_csv(csv_file)
images = dataset_table.values[:, 0]
labels = dataset_table.values[:, 2 : 2 + len(keypoint_keys)]

new_labels = []

for img_name, label_sequence in zip(images, labels):
    new_label_sequence = []
    img = cv2.imread(str(img_folder / img_name))
    h, w, c = img.shape
    for co_tuple in label_sequence:
        x, y = eval(co_tuple)
        rel_x = x / w
        rel_y = y / h

        new_label_sequence.append([rel_x, rel_y])
    new_labels.append(new_label_sequence)

with open(data_folder / "labels.json", "w") as f:
    json.dump(list(zip(images, new_labels)), f)
