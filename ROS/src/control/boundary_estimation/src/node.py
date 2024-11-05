from dataclasses import dataclass
from typing import List


@dataclass()
class Node:
    x: float
    y: float
    color: int  # 0: blue, 1: yellow, 2: other
    distance: float
    parent: "Node"
    children: List["Node"]
    angle: float
    turn: float
    cost: float
    id: int

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, other):
        if isinstance(other, Node):
            return self.id == other.id
        return False
