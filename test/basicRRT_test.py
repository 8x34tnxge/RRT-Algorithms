import numpy as np
from RRT.core import RRT
from RRT.algorithm import BasicRRT
from loguru import logger

logger.remove(0)
logger.add("test.log")

class TestBasicRRT:
    # test whether the node was added into tree successfully with attributes
    def test_merge(self):
        t1 = RRT(
            np.array([1]),
            np.array([1])
        )
        t2 = RRT(
            np.array([1]),
            np.array([1])
        )
        t1.addNode(np.array([1]))
        t1.addNode(np.array([3]))
        t1.addNode(np.array([2]))
        t1.addEdge(1, 2)
        t1.addEdge(1, 3)
        t2.addNode(np.array([2]))
        t2.addNode(np.array([4]))
        t2.addEdge(1, 2)
        newT = RRT.mergeFromTrees([t1, t2])
        logger.debug("now testing merge function")
        for n in newT.tree.nodes(data=True):
            logger.debug(f"node: {n}")
        for n in newT.tree.edges.data():
            logger.debug(f"edge: {n}")
        logger.debug(f"new tree info: {newT}")
        path, length = newT.getRoute(
            np.array([3]),
            np.array([4]),
        )
        logger.debug(f"path: {path}")
        logger.debug(f"length: {length}")
        path, length = newT.getRoute(
            np.array([3]),
            np.array([4]),
            'axisInfo'
        )
        logger.debug(f"path: {path}")
        logger.debug(f"length: {length}")

    def test_addNode(self):
        t = RRT(
            np.array([1]),
            np.array([1])
        )
        t.addNode(np.array([1]))
        t.addNode(np.array([2]))
        logger.debug("now testing add function")
        for n in t.tree.nodes(data=True):
            logger.debug(f"node: {n}")

    # test whether the node can be read directly with attributes
    def test_loadNode(self):
        t = RRT(
            np.array([1]),
            np.array([1])
        )
        t.addNode(np.array([1]))
        logger.debug("now testing load function")
        logger.debug(t.tree.nodes[1])
        logger.debug(t.tree.nodes[1]['coord'])
        logger.debug(t.tree.nodes[1]['coord'].shape)
    
    def test_addEdge(self):
        t = RRT(
            np.array([1]),
            np.array([1])
        )
        t.addNode(np.array([1]))
        t.addNode(np.array([2]))
        t.addEdge(1, 2)
        logger.debug("now testing edge function")
        for edge in t.tree.edges.data():
            logger.debug(edge)

    def test_loguru(self):
        logger.debug("test")