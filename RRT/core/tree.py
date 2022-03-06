from __future__ import annotations

from collections import namedtuple
from typing import Any, List

import networkx as nx
import numpy as np
from loguru import logger
from nptyping import NDArray
from RRT.core.info import RouteInfo
from RRT.util import array_hash, dist_calc


class RRT:
    def __init__(self, origin: NDArray[Any], target: NDArray[Any]):
        """the initial method of RRT

        Parameters
        ----------
        origin : NDArray[Any]
            the coordination info of the given origin
        target : NDArray[Any]
            the coordination info of the given target

        Raises
        ------
        ValueError
            the ndim of variable [origin, target] is not equal
        """
        self.IDcounter: int = 0
        if origin.shape[0] != target.shape[0]:
            raise ValueError("Origin's dimension must be equal to target's dimension")
        self.ndim: int = origin.shape[0]
        self.origin: NDArray[Any] = origin
        self.target: NDArray[Any] = target
        # use graph to replace the tree structure
        self.tree: nx.classes.graph.Graph = nx.Graph()
        self.add_node(origin)

        self.is_reach_target: bool = False

    def get_nodes(self) -> nx.classes.reportviews.NodeDataView:

        """the instance method to get the nodes from the tree

        Returns
        -------
        nx.classes.reportviews.NodeDataView
            the node infomation in tree
        """
        return self.tree.nodes.data()

    def get_edges(self) -> nx.classes.reportviews.EdgeDataView:
        """the instance method to get the edges from the tree

        Returns
        -------
        nx.classes.reportviews.EdgeDataView
            the edge information in tree
        """
        return self.tree.edges.data()

    def get_nearest_neighbors(self, node_info: NDArray[Any], num: int = 1) -> List[int]:
        """the instance method to get nearest neighbors according to the given node coordination info

        Parameters
        ----------
        node_info : NDArray[Any]
            the coordination info of the given node
        num : int
            the number of the nearest neighbors

        Returns
        -------
        List[int]
            the ID of the nearest neighbor point(s)/node(s)
        """
        assert num > 0

        rel_info = namedtuple("rel_info", ("node_id", "dist"))
        info_list = []
        for id, info in self.get_nodes():
            coord_info = info["coord"]
            dist = dist_calc(coord_info, node_info)
            info_list.append(rel_info(id, dist))

        ret = []
        num = num if num < len(info_list) else len(info_list)
        info_list.sort(key=lambda x: x.dist)
        for idx in range(num):
            ret.append(info_list[idx].node_id)

        return ret

    def get_route(
        self,
        origin: NDArray[Any] = None,
        target: NDArray[Any] = None,
    ) -> RouteInfo:
        """the instance method to get the route information including route sequence and route length

        Parameters
        ----------
        origin : NDArray[Any], optional
            the origin information (coordination info), by default None
        target : NDArray[Any], optional
            the target information (coordination info), by default None

        Returns
        -------
        RouteInfo
            the route information including route sequence, route coordination and route total length

        Raises
        ------
        AttributeError
            The given origin is not found in trees
        AttributeError
            The given target is not found in trees
        """
        if origin is None:
            origin = self.origin

        if target is None:
            target = self.target

        nodes_info = self.get_nodes()
        originID, targetID = None, None
        for nodeID, node_info in nodes_info:
            if all(origin == node_info["coord"]):
                originID = nodeID
            if all(target == node_info["coord"]):
                targetID = nodeID
        if originID is None:
            raise AttributeError("The given origin is not found in trees")
        if targetID is None:
            raise AttributeError("The given target is not found in trees")

        try:
            route = nx.algorithms.shortest_path(
                self.tree, originID, targetID, weight="weight"
            )
        except nx.exception.NetworkXNoPath:
            return RouteInfo(None, None, None)

        axis_info = []
        for nodeID in route:
            coord_info = nodes_info[nodeID]["coord"]
            axis_info.append(coord_info)
            # for dimID in range(self.ndim):
            #     axis_info[dimID].append(coord_info[dimID])

        length = nx.algorithms.shortest_path_length(
            self.tree, originID, targetID, weight="weight"
        )
        return RouteInfo(route, axis_info, np.float64(length))

    @classmethod
    def merge_from_trees(
        cls, trees: List[RRT], origin: NDArray[Any], target: NDArray[Any]
    ) -> RRT:
        """the class method to merge a list of RRTs to a new RRT

        Parameters
        ----------
        trees : List[RRT]
            the list of RRTs to merge
        origin : NDArray[Any]
            the coordination info of the given origin
        target : NDArray[Any]
            the coordination info of the given target

        Returns
        -------
        RRT
            new RRT merged from the given list of RRTs
        """
        new_tree = cls(origin, target)
        attr_search_dict = {}
        for tree in trees:
            table4ID = {}
            # insert each point/node from the given trees into the new tree
            # [x] check node existence
            for node in tree.tree.nodes(data=True):
                oldID, node_info = node
                if array_hash(node_info["coord"]) in attr_search_dict.keys():
                    table4ID[oldID] = attr_search_dict[array_hash(node_info["coord"])]
                    continue

                newID = new_tree.add_node(node_info["coord"])

                # update hash table
                attr_search_dict[array_hash(node_info["coord"])] = newID
                table4ID[oldID] = newID

            # insert each edge from the given trees into the new tree
            for edge in tree.tree.edges.data():
                prevID = table4ID[edge[0]]
                postID = table4ID[edge[1]]
                weight = edge[-1]["weight"]
                new_tree.add_edge(prevID, postID, weight)

        new_tree.update_status()
        return new_tree

    def add_node(self, node_info: NDArray[Any]) -> int:
        """the instance method to add a node to the tree and return its ID

        Parameters
        ----------
        node_info : NDArray[Any]
            the info of the new point/node

        Returns
        -------
        int
            the ID of this new point/node
        """
        # prevent duplicated node from adding
        nodes = self.get_nodes()
        for node in nodes:
            if all(node_info == node[-1]["coord"]):
                return node[0]

        self.IDcounter += 1
        nodeID = self.IDcounter
        # if all(node_info == self.target):
        #     self.is_reach_target = True
        self.tree.add_node(nodeID, coord=node_info)
        return nodeID

    def add_edge(self, currID: int, newID: int, weight: np.float64 = None) -> None:
        """the instance method to add a new edge to the tree

        Parameters
        ----------
        currID : int
            the node ID of one end of the edge
        newID : int
            the node ID of another end of the edge
        weight : np.float64, optional
            the weight/length between the given two nodes, by default None
        """
        weight = (
            weight
            if weight is not None
            else dist_calc(
                prev_node_coord_info=self.tree.nodes[currID]["coord"],
                post_node_coord_info=self.tree.nodes[newID]["coord"],
            )
        )

        self.tree.add_edge(currID, newID, weight=weight)

    def update_status(self):
        """the instance method to update reach status
        """
        try:
            route_info = self.get_route()
            if route_info.get_length() == None:
                self.is_reach_target = False
            else:
                self.is_reach_target = True
        except AttributeError :
            self.is_reach_target = False