from typing import Any

import numpy as np
from nptyping import NDArray


def calc_unit_vector(
    head_coord_info: NDArray[Any], tail_coord_info: NDArray[Any]
) -> NDArray[Any]:
    """calc the unit vector from head to tail vector

    Parameters
    ----------
    head_coord_info : NDArray[Any]
        the head vector
    tail_coord_info : NDArray[Any]
        the tail vector

    Returns
    -------
    NDArray[Any]
        the unit vector from head to tail vector
    """
    delta_vector: NDArray[Any] = tail_coord_info - head_coord_info
    vector_magnitude = np.linalg.norm(delta_vector)
    if vector_magnitude == 0:
        return np.zeros(delta_vector.shape)
    unit_vector = delta_vector / vector_magnitude

    return unit_vector
