import numpy as np
from RRT.algorithm import BasicRRT
from loguru import logger

logger.add("test.log")

class TestBasicRRT:
    # test whether the node was added into tree successfully with attributes
    def test_addNode(self):
        t = BasicRRT(
            np.array([1]),
            np.array([1])
        )
        t.addNode(np.array([1]))
        t.addNode(np.array([2]))
        for n in t.tree.nodes(data=True):
            logger.debug(f"node: {n}")

    # test whether the node can be read directly with attributes
    def test_loadNode(self):
        t = BasicRRT(
            np.array([1]),
            np.array([1])
        )
        logger.debug(t.tree.nodes[1])
        logger.debug(t.tree.nodes[1]['coord'])
        logger.debug(t.tree.nodes[1]['coord'].shape)