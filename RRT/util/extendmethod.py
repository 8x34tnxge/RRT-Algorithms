from RRT.core import RRT as RRT
import numpy as np


def directly_extend(tree: RRT, new_node_info: np.ndarray, nearest_node_ID: int):
    new_node_ID = tree.addNode(new_node_info)
    tree.addEdge(nearest_node_ID, new_node_ID)


