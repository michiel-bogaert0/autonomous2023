#!/usr/bin/python
import argparse
import re

import cv2.aruco as aruco
import numpy as np

"""
A generic validator class to be able to validate regex based expressions
"""


class Validator(object):

    """
    Initializes the class

    Args:
      pattern: the regex pattern to check against
    """

    def __init__(self, pattern):
        self._pattern = re.compile(pattern)

    """
      Actually validates against said pattern

      Args:
        value: the value that must match the pattern

      Returns:
        The parameter 'value' when value matches the pattern. Throws an error if this is not the case (which will be visible in the terminal)
    """

    def __call__(self, value):
        if not self._pattern.match(value):
            raise argparse.ArgumentTypeError(
                f"Argument has to match {self._pattern.pattern}"
            )
        return value


def main():
    gridSize = Validator(r"([0-9]{1,})x([0-9]{1,})")
    jpg_or_png = Validator(r"([^\s]+(\.(jpg)|(png))$)")
    npz = Validator(r"([^\s]+(\.(npz))$)")
    board_type = Validator(r"chess|charuco")

    parser = argparse.ArgumentParser(description="Camera calibration")

    parser.add_argument(
        "img",
        type=jpg_or_png,
        help="image mask for calibration (for example, cal_*.jpg)",
    )
    parser.add_argument("size", type=gridSize, help="")
    parser.add_argument(
        "square_length", type=float, help="The real length of a side of a square in mm"
    )

    parser.add_argument(
        "marker_length",
        type=float,
        help="The real length of a side of a marker in mm (only relevant when using charuco boards)",
    )

    parser.add_argument(
        "out", type=npz, help="output path for the resulting calibration data"
    )

    parser.add_argument(
        "board_type",
        type=board_type,
        help="Type of board. Choice between 'chess' or 'charuco'",
    )

    args = parser.parse_args()

    # Start the actual program
    from calibrate import Calibrator

    calibrator = Calibrator(args.size, args.square_length, args.marker_length)

    if args.board_type == "chess":
        mtx, dist = calibrator.chessboard_calibrate(args.img)
    elif args.board_type == "charuco":
        mtx, dist = calibrator.charucoboard_calibrate(args.img, aruco.DICT_5X5_50)
    else:
        return

    np.savez(args.out, camera_matrix=mtx, distortion_matrix=dist)


if __name__ == "__main__":
    main()
