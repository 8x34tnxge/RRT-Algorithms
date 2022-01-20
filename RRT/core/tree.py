from __future__ import annotations

from typing import List

import networkx as nx
import numpy as np
from RRT.util import distCalc


class RRT:
    def __init__(self, origin: np.ndarray, target: np.ndarray) -> None:
        self.IDcounter: int = 0
        self.origin: np.ndarray = origin
        self.target: np.ndarray = target
        # use graph to replace the tree structure
        self.tree = nx.Graph()

    def mergeFromTrees(cls, trees: List[RRT]) -> RRT:
        for tree in trees:
            for node in tree.tree.nodes(data=True):
                cls.addNode(node)
            for edge in tree.tree.edges:
                pass

    def addNode(self, nodeInfo: np.ndarray) -> int:
        """the class method to add a node to the tree and return its ID

        Args:
            nodeInfo (np.ndarray): the info of the new point/node

        Returns:
            int: the ID of this new point/node
        """
        self.IDcounter += 1
        nodeID = self.IDcounter
        self.tree.add_node(nodeID, coord=nodeInfo)
        return nodeID

    def addEdge(self, currID, newID) -> None:
        self.tree.add_edge(currID, newID, weight=distCalc(
            prevNodeCoordInfo=self.tree.nodes[currID],
            postNodeCoordInfo=self.tree.nodes[newID]
        ))
