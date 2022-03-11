import numpy as np


def array_hash(array: np.ndarray) -> str:
    """the method to convert an array/ndarray to a hashable object (str)

    Parameters
    ----------
    array : np.ndarray
        the array to convert

    Returns
    -------
    str
        the result of the conversion
    """
    return str(array)
