from RRT.core import RRT_Template
import numpy as np
import networkx as nx

class basicRRT(RRT_Template):
    def __init__(self, origin: np.ndarray, target: np.ndarray) -> None:
        super().__init__(origin, target)