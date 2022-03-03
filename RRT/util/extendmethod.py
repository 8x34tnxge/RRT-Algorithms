import numpy as np
from RRT.core import RRT as RRT


def directly_extend(tree: RRT, new_node_info: np.ndarray, nearest_node_ID: int):
    """the method to directly extend the tree from nearest node using new node

    Parameters
    ----------
    tree : RRT
        the tree storing the node and edge info
    new_node_info : np.ndarray
        the info of node to be inserted to tree
    nearest_node_ID : int
        the id of the nearest node from new node
    """
    new_node_ID = tree.add_node(new_node_info)
    tree.add_edge(nearest_node_ID, new_node_ID)
