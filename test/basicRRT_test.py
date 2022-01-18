import numpy as np
from RRT.algorithm import basicRRT

def test_basicRRT():
    t = basicRRT(
        np.array([1]),
        np.array([1])
    )