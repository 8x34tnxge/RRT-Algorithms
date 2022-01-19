from __future__ import annotations

from abc import ABC, abstractclassmethod, abstractmethod, abstractstaticmethod
from typing import List

import networkx as nx
import numpy as np
from RRT.util import distCalc


class RRT_Template(ABC):
    def __init__(self, origin: np.ndarray, target: np.ndarray):
        self.IDcounter = 0
        self.origin = origin
        self.target = target
        # use graph to replace the tree structure
        self.tree = nx.Graph()

    @abstractclassmethod
    def mergeFromTrees(cls, trees: List[RRT_Template]) -> RRT_Template:
        pass

    def addNode(self, nodeInfo) -> None:
        self.IDcounter += 1
        self.tree.add_node(self.IDcounter, coord=nodeInfo)

    def addEdge(self, nodeIDpair) -> None:
        prevID, postID = nodeIDpair
        
        self.tree.add_edge(prevID, postID, weight=distCalc(
            prevNodeCoordInfo=self.tree.nodes[prevID],
            postNodeCoordInfo=self.tree.nodes[postID]
        ))
