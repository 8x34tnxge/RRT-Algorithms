from __future__ import annotations

from typing import List

import numpy as np
from RRT.core.route_info import RouteInfo
from RRT.core.tree_node import TreeNode
from RRT.util import dist_calc


class Tree:
    def __init__(self, origin_coord):
        self.root = TreeNode(origin_coord)
        self.nodes = [self.root]

    def get_node(self, node: TreeNode) -> TreeNode:
        assert self.is_reach(node)

        return self.nodes[self.nodes.index(node)]

    def add_node(self, coord, parent):
        dist = dist_calc(parent.coord, coord)
        new_node = TreeNode(coord, parent.cost + dist, parent)

        if new_node in self.nodes:
            node = self.nodes[self.nodes.index(new_node)]
            return node

        self.nodes.append(new_node)

        return new_node

    def get_route(self, end_node: TreeNode) -> RouteInfo:
        route = [end_node]
        cost = end_node.cost

        while route[-1].parent != None:
            route.append(route[-1].parent)
        route.reverse()
        ret = RouteInfo(route, cost)

        return ret

    def get_nearest_neighbors(self, coord, n=1) -> List[TreeNode]:
        n = n if 0 < n <= len(self.nodes) else len(self.nodes)

        ret = []
        for node in self.nodes:
            dist = dist_calc(node.coord, coord)
            ret.append(dist)
        idx = np.argpartition(np.array(ret), n - 1)

        return [self.nodes[idx[x]] for x in range(n)], np.array([ret[idx[x]] for x in range(n)])

    def update_cost(self):
        for node in self.nodes:
            node.cost = np.inf
        for node in self.nodes:
            node.update_cost()

    def is_reach(self, node: TreeNode):
        return node in self.nodes
