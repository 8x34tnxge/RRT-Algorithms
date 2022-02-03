from __future__ import annotations

from typing import List, Tuple, Any

from nptyping import NDArray
import networkx as nx
import numpy as np
from RRT.util import distCalc, arrayHash

# [x] the method to check whether to reach the target
# [x] the method to get nearestNodeID
# [x] the method to get the shortest route from origin to target
class RRT:
    """Randomly-exploring Random Tree Structure"""
    ######################################
    ############# properties #############
    ######################################
    ndim: int
    origin: NDArray[Any]
    target: NDArray[Any]
    tree: nx.classes.graph.Graph
    isReachTarget: bool

    #########################################
    ############# class methods #############
    #########################################
    def __init__(self, origin: NDArray[Any], target: NDArray[Any]) -> None:
        """the initial method of RRT

        Args:
            origin (NDArray[Any]): the coordination info of the given origin
            target (NDArray[Any]): the coordination info of the given target
        """
        self.IDcounter: int = 0
        if origin.ndim != target.ndim:
            raise ValueError("Origin's dimension must be equal to target's dimension")
        self.ndim: int = origin.ndim
        self.origin: NDArray[Any] = origin
        self.target: NDArray[Any] = target
        # use graph to replace the tree structure
        self.tree = nx.Graph()
        self.addNode(origin)

        self.isReachTarget = False

    def getNodes(self) -> nx.classes.reportviews.NodeDataView:
        """the method to get the nodes from the tree

        Returns:
            nx.classes.reportviews.NodeDataView: the nodes info
        """
        return self.tree.nodes.data()

    def getEdges(self) -> nx.classes.reportviews.EdgeDataView:
        """the method to get the edges from the tree

        Returns:
            nx.classes.reportviews.NodeDataView: the edges info
        """
        return self.tree.edges.data()

    def getNearestNeighbors(self, nodeInfo: NDArray[Any]) -> int:
        """the class method to get nearest neighbors according to the given node coordination info

        Args:
            nodeInfo (NDArray[Any]): the coordination info of the given node

        Returns:
            int: the ID of the nearest neighbor point/node
        """
        nearestNodeID = 0
        minDist = np.Infinity

        for id, info in self.getNodes():
            coordInfo = info['coord']
            dist = np.linalg.norm(nodeInfo, coordInfo)
            if dist < minDist:
                minDist = dist
                nearestNodeID = id

        return nearestNodeID

    def getRoute(
        self,
        origin: NDArray[Any] = None,
        target: NDArray[Any] = None,
        routeType: str = "nodeID",
    ) -> Tuple[List, np.float64]:
        """the method to get the route information including route sequence and route length

        Args:
            origin (NDArray[Any], optional): the origin information (coordination info). Defaults to None.
            target (NDArray[Any], optional): the target information (coordination info). Defaults to None.
            routeType (str, optional): the type of route sequence,
            'nodeID' will return a list of node IDs: [1, 2, 3] for example
            'axisInfo' will return a list containing x-coord & y-coord of nodes in order: [ [1., 2., 3.], [1., 2., 3.] ] for example.
            Defaults to 'nodeID'.

        Raises:
            AttributeError: The given origin is not found in trees
            AttributeError: The given target is not found in trees

        Returns:
            Tuple[List, np.float64]: route sequence and its length respectively
        """
        if origin is None:
            origin = self.origin

        if target is None:
            target = self.target

        nodesInfo = self.getNodes()
        originID, targetID = None, None
        for nodeID, nodeInfo in nodesInfo:
            if all(origin == nodeInfo["coord"]):
                originID = nodeID
            if all(target == nodeInfo["coord"]):
                targetID = nodeID
        if originID is None:
            raise AttributeError("The given origin is not found in trees")
        if targetID is None:
            raise AttributeError("The given target is not found in trees")

        route = nx.algorithms.shortest_path(
            self.tree, originID, targetID, weight="weight"
        )
        length = nx.algorithms.shortest_path_length(
            self.tree, originID, targetID, weight="weight"
        )

        if routeType is "nodeID":
            return route, length
        elif routeType is "axisInfo":
            axisInfo = [[] for _ in range(self.ndim)]
            for nodeID in route:
                coordInfo = nodesInfo[nodeID]['coord']
                for dimID in range(self.ndim):
                    axisInfo[dimID].append(coordInfo[dimID])

            return axisInfo, length

        else:
            raise AttributeError("the param [nodeType] is not 'nodeID" or 'axisInfo')

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
            # [x] check node existence
            for node in tree.tree.nodes(data=True):
                oldID, nodeInfo = node
                if arrayHash(nodeInfo["coord"]) in attrSearchDict.keys():
                    table4ID[oldID] = attrSearchDict[arrayHash(nodeInfo["coord"])]
                    continue

                newID = newTree.addNode(nodeInfo["coord"])

                # update hash table
                attrSearchDict[arrayHash(nodeInfo["coord"])] = newID
                table4ID[oldID] = newID

            # insert each edge from the given trees into the new tree
            for edge in tree.tree.edges.data():
                prevID = table4ID[edge[0]]
                postID = table4ID[edge[1]]
                weight = edge[-1]["weight"]
                newTree.addEdge(prevID, postID)

        return newTree

    def addNode(self, nodeInfo: NDArray[Any]) -> int:
        """the class method to add a node to the tree and return its ID

        Args:
            nodeInfo (NDArray[Any]): the info of the new point/node

        Returns:
            int: the ID of this new point/node
        """
        self.IDcounter += 1
        nodeID = self.IDcounter
        if str(nodeInfo) == str(self.target):
            self.isReachTarget = True
        self.tree.add_node(nodeID, coord=nodeInfo)
        return nodeID

    def addEdge(self, currID, newID) -> None:
        """the class method to add a new edge to the tree

        Args:
            currID ([type]): the node ID of one end of the edge
            newID ([type]): the node ID of another end of the edge
        """
        self.tree.add_edge(
            currID,
            newID,
            weight=distCalc(
                prevNodeCoordInfo=self.tree.nodes[currID]["coord"],
                postNodeCoordInfo=self.tree.nodes[newID]["coord"],
            ),
        )
