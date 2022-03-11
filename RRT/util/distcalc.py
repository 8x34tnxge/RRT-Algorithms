import numpy as np


def dist_calc(
    prev_node_coord_info: np.ndarray, post_node_coord_info: np.ndarray
) -> np.ndarray:
    """the method to calc distance between two points/nodes in arbitrary dimensional space

    Parameters
    ----------
    prev_node_coord_info : np.ndarray
        the coordination of point/node of one end in arbitrary dimensional space
    post_node_coord_info : np.ndarray
        the coordination of point/node of the other end in arbitrary dimensional space

    Returns
    -------
    np.ndarray
        the Euclidean Distance between the given points/nodes
    """
    delta_coord = prev_node_coord_info - post_node_coord_info
    distance = np.linalg.norm(delta_coord)

    return distance
