from argparse_dataclass import dataclass


@dataclass
class RektNetTrainParam:
    val_ratio: float = 0.15
    test_ratio: float = 0.15
