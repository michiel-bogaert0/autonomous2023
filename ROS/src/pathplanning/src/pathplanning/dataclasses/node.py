from dataclasses import dataclass
from typing import List


@dataclass()
class Node:
    x: float
    y: float
    distance: float
    parent: "Node"
    children: List["Node"]
    angle: float
    angle_change: float
