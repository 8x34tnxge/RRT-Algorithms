from __future__ import annotations

from typing import List

import networkx as nx
import numpy as np
from RRT.util import distCalc, arrayHash

class RRT:
    """Randomly-exploring Random Tree Structure
    """
    def __init__(self, origin: np.ndarray, target: np.ndarray) -> None:
        self.IDcounter: int = 0
        self.origin: np.ndarray = origin
        self.target: np.ndarray = target
        # use graph to replace the tree structure
        self.tree = nx.Graph()
    
    def getNodes(self):
        return self.tree.nodes.data()

    def getEdges(self):
        return self.tree.edges.data()

    @classmethod
    def mergeFromTrees(cls, trees: List[RRT]) -> RRT:
        """merge a list of RRT to a new RRT

        Args:
            trees (List[RRT]): the list of RRTs to merge

        Returns:
            RRT: new RRT merged from the given list of RRTs
        """
        newTree = cls(trees[0].origin, trees[1].target)
        attrSearchDict = {}
        for tree in trees:
            table4ID = {}
            # insert each point/node from the given trees into the new tree
            #[x] check node existence
            for node in tree.tree.nodes(data=True):
                oldID, nodeInfo = node
                if arrayHash(nodeInfo['coord']) in attrSearchDict.keys():
                    table4ID[oldID] = attrSearchDict[arrayHash(nodeInfo['coord'])]
                    continue

                newID = newTree.addNode(nodeInfo['coord'])

                # update hash table
                attrSearchDict[arrayHash(nodeInfo['coord'])] = newID
                table4ID[oldID] = newID

            # insert each edge from the given trees into the new tree
            for edge in tree.tree.edges.data():
                prevID = table4ID[edge[0]]
                postID = table4ID[edge[1]]
                weight = edge[-1]['weight']
                newTree.addEdge(prevID, postID)

        return newTree

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
        """the class method to add a new edge to the tree

        Args:
            currID ([type]): the node ID of one end of the edge
            newID ([type]): the node ID of another end of the edge
        """
        self.tree.add_edge(currID, newID, weight=distCalc(
            prevNodeCoordInfo=self.tree.nodes[currID]['coord'],
            postNodeCoordInfo=self.tree.nodes[newID]['coord']
        ))
