from __future__ import annotations

from RRT.util.distcalc import dist_calc
import numpy as np


class TreeNode:
    def __init__(self, coord, cost=0, p: TreeNode = None):
        self.coord = coord
        self.real_dist = None
        self.cost = cost
        self.parent = p

    def __eq__(self, other: TreeNode):
        if isinstance(other, TreeNode) and all(self.coord == other.coord):
            return True
        return False

    def update_cost(self):
        # need to set cost to np.inf before update to reduce recalculate time
        if np.isfinite(self.cost):
            return self.cost
        elif self.parent is None:
            self.cost = 0
        else:
            self.cost = self.parent.update_cost() + dist_calc(
                self.parent.coord, self.coord
            )

        return self.cost
