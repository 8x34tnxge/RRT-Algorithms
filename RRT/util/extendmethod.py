from RRT.core import RRT as RRT
import numpy as np
from loguru import logger


def directly_extend(tree: RRT, new_node_info: np.ndarray, nearest_node_ID: int):
    logger.debug(new_node_info)
    new_node_ID = tree.add_node(new_node_info)
    tree.add_edge(nearest_node_ID, new_node_ID)


