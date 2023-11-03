from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from aggregation.aggregation import Aggregation
from std_msgs.msg import Float32


class plus_aggr(Aggregation):
    """
    example aggregation class
    publishes the sum of two outputs
    """

    def __init__(self):
        super().__init__("plus")

    def aggregate_data(self, x: Float32, y: Float32):
        return Float32(data=x.data + y.data)

    def get_output_type(self):
        return Float32

    def plot(self, values, savedir):
        p = []
        for v in values:
            p.append(v.data)
        plt.plot(np.arange(len(p)), p)
        plt.savefig(savedir)

    def update_state(self, input: Any) -> Any:
        return super().update_state(input)
