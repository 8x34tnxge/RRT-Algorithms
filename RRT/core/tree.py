from __future__ import annotations

from abc import ABC, abstractclassmethod, abstractmethod, abstractstaticmethod
from typing import List

import networkx as nx
import numpy as np


class RRT_Template(ABC):
    def __init__(self, origin: np.ndarray, target: np.ndarray):
        # use graph to replace the tree structure
        self.origin = origin
        self.target = target
        self.tree = nx.Graph()

    @abstractclassmethod
    def mergeFromTrees(cls, trees: List[RRT_Template]) -> RRT_Template:
        pass

    @abstractmethod
    def insertNode(self, node):
        pass

    @abstractmethod
    def insertNodes(self, nodes):
        pass

    @abstractmethod
    def getNearestNodes(self, node, num):
        pass

