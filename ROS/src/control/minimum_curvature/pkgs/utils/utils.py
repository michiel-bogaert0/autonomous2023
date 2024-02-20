import math
from typing import List, Tuple

import numpy as np
from scipy.interpolate import interp1d, splev, splprep


def generate_center_points(blue_cones, yellow_cones) -> List[Tuple[float, float]]:
    refPoints = []
    c = 0
    for i, yellow_cone in enumerate(yellow_cones):
        min_distance = math.inf
        nearest_blue = None
        removed = 0
        for j, blue_cone in enumerate(blue_cones[c:]):
            distance = math.sqrt(
                (blue_cone.x() - yellow_cone.x()) ** 2
                + (blue_cone.y() - yellow_cone.y()) ** 2
            )
            if distance < min_distance:
                min_distance = distance
                for index, el in enumerate(blue_cones[c + removed : c + j]):
                    if index + removed > 0 or i == 0:
                        mindis = math.inf
                        closest = None
                        for yc in yellow_cones[: i + 1]:
                            dis = math.sqrt(
                                (el.x() - yc.x()) ** 2 + (el.y() - yc.y()) ** 2
                            )
                            if dis < mindis:
                                mindis = dis
                                closest = yc
                        refPoints.append((el + closest) / 2)
                removed = j
                nearest_blue = blue_cone
        if nearest_blue:
            refPoints.append((yellow_cone + nearest_blue) / 2)

        c += removed

    return refPoints


# Generate interpolated points along the curvilinear path
def generate_interpolated_points(points) -> np.array[Tuple[float, float]]:
    path = np.array([[p[0], p[1]] for p in points])

    per = 1  # BSPline periodicity, 0 = not periodic, 1 = periodic

    # Linear interpolation between center points to add more points for BSpline smoothing
    distance = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
    distance = np.insert(distance, 0, 0) / distance[-1]

    alpha = np.linspace(0, 1, len(path) * 3)
    interpolator = interp1d(distance, path, kind="linear", axis=0)
    path = interpolator(alpha)

    # Smooth path with BSpline interpolation
    path = path.T  # Transpose to get correct shape for BSpline, splprep expects (2, N)
    w = np.array(
        [1] * len(path[0])
    )  # Weights for BSpline (Here, same weight for every point)

    tck, u = splprep(path, w=w, s=10, per=per)  # Calculate BSpline
    spline_path = np.array(
        splev(u, tck)
    ).T  # Evaluate BSpline and transpose back to (N, 2)

    return spline_path
