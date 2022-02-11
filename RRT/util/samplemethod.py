import math
import random
from typing import List, Union

import numpy as np


def random_sample(
    min_border: Union[List[float], np.ndarray], max_border: Union[List[float], np.ndarray]
) -> np.ndarray:
    """the method to randomly sample a point/node in a square space with max and min value for each dimension

    Parameters
    ----------
    min_border : Union[List[float], np.ndarray]
        the list of min values of dimensions
    max_border : Union[List[float], np.ndarray]
        the list of max values of dimensions

    Returns
    -------
    np.ndarray
        the coord info of the sampled point/node
    """
    sample_coord = []
    for minVal, maxVal in zip(min_border, max_border):
        sample_coord.append(random.randint(minVal, maxVal))

    # convert list to array for better math manipulation
    sample_coord = np.array(sample_coord)

    return sample_coord


def sample_unit_ball(num: int, dim: int=3) -> List[np.ndarray]:
    """the method to get a number of points/nodes in a unit ball for given dimensions

    Parameters
    ----------
    num : int
        the number of points/nodes needed to be sampled
    dim : int, optional
        the dimension of the unit ball, by default 3

    Returns
    -------
    List[np.ndarray]
        the list of sampled points/nodes

    Raises
    ------
    ValueError
        current version of this method only support 2 & 3 dim
    """
    assert num > 0
    assert isinstance(num, int)

    samples = []

    for _ in range(num):
        if dim == 2:
            sample = get2D_sample()
        elif dim == 3:
            sample = get3D_sample()
        else:
            raise ValueError("Parameter [dim] only support 2 or 3")

        samples.append(sample)

    return samples


def get2D_sample() -> np.ndarray:
    """the method to sample one point/node in 2-dimensional unit ball

    Returns
    -------
    np.ndarray
        the coord info of the sampled point/node
    """
    # let's set r, theta
    # 0 < r <= 1, 0 < theta < 2pi
    r = random.random()
    theta = 2 * math.pi * random.random()

    # then, x = r * cos(theta)
    # y = r * sin(theta)
    x = r * math.cos(theta)
    y = r * math.sin(theta)

    sample = np.ndarray([x, y])
    return sample


def get3D_sample() -> np.ndarray:
    """the method to sample one point/node in 3-dimensional unit ball

    Returns
    -------
    np.ndarray
        the coord info of the sampled point/node
    """
    # let's set r, theta, phi
    # 0 < r <= 1, 0 < theta < 2pi, 0 < phi < pi
    r = random.random()
    theta = random.random() * 2 * math.pi
    phi = random.random() * math.pi

    # then, x = r * cos(theta) * sin(phi)
    # y = r * sin(theta) * sin(phi)
    # z = r * cos(phi)
    x = r * math.cos(theta) * math.sin(phi)
    y = r * math.sin(theta) * math.sin(phi)
    z = r * math.cos(phi)

    sample = np.array([x, y, z])
    return sample
