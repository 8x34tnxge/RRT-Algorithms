from RRT.core import RRT as RRT
import numpy as np


def directlyExtend(tree: RRT, newNodeInfo: np.ndarray, nearestNodeID: int):
    newNodeID = tree.addNode(newNodeInfo)
    tree.addEdge(nearestNodeID, newNodeID)


