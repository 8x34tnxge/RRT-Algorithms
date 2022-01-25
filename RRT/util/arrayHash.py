import numpy as np

def arrayHash(array: np.ndarray) -> str:
    """the method to convert an array/ndarray to a hashable object (str)

    Args:
        array (np.ndarray): the array to convert

    Returns:
        Tuple: the result of the conversion
    """
    return str(array)