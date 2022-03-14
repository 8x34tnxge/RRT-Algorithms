from typing import Any

import numpy as np
import scipy.interpolate as si
from nptyping import NDArray


# reference: [https://stackoverflow.com/questions/34803197/fast-b-spline-algorithm-with-numpy-scipy]
def bspline(cv, n=100, degree=3, periodic=False):
    """Calculate n samples on a bspline

    cv :      Array ov control vertices
    n  :      Number of samples to return
    degree:   Curve degree
    periodic: True - Curve is closed
              False - Curve is open
    """

    # If periodic, extend the point array by count+degree+1
    cv = np.asarray(cv)
    count = len(cv)

    if periodic:
        factor, fraction = divmod(count + degree + 1, count)
        cv = np.concatenate((cv,) * factor + (cv[:fraction],))
        count = len(cv)
        degree = np.clip(degree, 1, degree)

    # If opened, prevent degree from exceeding count-1
    else:
        degree = np.clip(degree, 1, count - 1)

    # Calculate knot vector
    kv = None
    if periodic:
        kv = np.arange(0 - degree, count + degree + degree - 1)
    else:
        kv = np.clip(np.arange(count + degree + 1) - degree, 0, count - degree)

    # Calculate query range
    u = np.linspace(periodic, (count - degree), n)

    # Calculate result
    return np.array(si.splev(u, (kv, cv.T, degree))).T


# type definition should be changed or combined
def path_smooth_with_bspline(
    origin_path: NDArray[Any], degree: int = 2, fill_num: int = 100
) -> NDArray[Any]:
    # [ ] N, s and k is better to be params
    new_path = bspline(origin_path, n=fill_num, degree=2)

    return new_path
