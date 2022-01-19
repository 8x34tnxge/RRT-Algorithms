import numpy as np

def distCalc(prevNodeCoordInfo: np.ndarray, postNodeCoordInfo: np.ndarray) -> np.ndarray:
    """ the method to calc distance between two points/nodes in arbitrary dimensional space

    Args:
        prevNodeCoordInfo (np.ndarray): the coordination of point/node of one end in arbitrary dimensional space
        postNodeCoordInfo (np.ndarray): the coordination of point/node of the other end in arbitrary dimensional space

    Returns:
        np.ndarray: the Euclidean Distance between the given points/nodes
    """
    deltaCoord = prevNodeCoordInfo - postNodeCoordInfo
    distance = np.linalg.norm(deltaCoord)

    return distance