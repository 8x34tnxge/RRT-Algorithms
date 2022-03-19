from __future__ import annotations

from typing import List

import numpy as np
from RRT.algorithm.RRT_template import RRT_Template
from RRT.core.info import DroneInfo
from RRT.core.mission_info import MissionInfo
from RRT.core.route_info import RouteInfo
from RRT.core.sign import Status
from RRT.core.tree import Tree, TreeNode
from RRT.util.distcalc import dist_calc
from RRT.util.samplemethod import steer


class RRT_Star(RRT_Template):
    """basic Rapidly-exploring Random Tree algorithm with only one forward-search tree"""

    def __init__(
        self,
        drone_info: DroneInfo,
        mission_info: MissionInfo,
        explore_prob: np.float64,
        step_size: np.float64,
        neighbor_num: int = 5,
        max_attempts: np.int32 = np.inf,
    ):
        """the init method of RRT

        Parameters
        ----------
        drone_info : DroneInfo
            the drone infomation
        mission_info : MissionInfo
            the mission infomation
        explore_prob : np.float64
            the probability of exploration (1 - the probability of going forward target)
        step_size : np.float64
            the size/length of each step
        max_attempts : np.int32, optional
            the maximum number of attempts, by default np.inf
        """
        super().__init__(
            drone_info, mission_info, explore_prob, step_size, max_attempts
        )
        self.neighbor_num = neighbor_num
        self.search_tree: Tree = Tree(mission_info.origin)

    def run(self) -> bool:
        """the method to run the basic RRT algorithm

        Returns
        -------
        bool
            whether basic RRT algorithm reach the target from origin
        """
        attempt_cnt = 0
        while True:
            # loop ctrl condition
            if np.isfinite(self.max_attempts) and attempt_cnt > self.max_attempts:
                break
            elif np.isinf(self.max_attempts) and self.final_ret is not None:
                break

            # loop start
            attempt_cnt += 1

            new_sample = self.sample()
            neighbors, neighbor_dist = self.search_tree.get_nearest_neighbors(
                new_sample, self.neighbor_num
            )
            new_sample = steer(neighbors[np.argmin(neighbor_dist)].coord, new_sample, self.step_size)

            collision_free_list = self.neighbor_collision_free(new_sample, neighbors)
            parent = self.choose_parent(new_sample, neighbors, collision_free_list)
            if parent is None:
                continue
            sample_node = self.search_tree.add_node(new_sample, parent)

            if sample_node == TreeNode(self.mission_info.target):
                target_node = self.search_tree.get_node(sample_node)
                self.final_ret = self.search_tree.get_route(target_node)

            self.rewire(sample_node, neighbors, collision_free_list)

        if self.final_ret is not None:
            return Status.Success
        return Status.Failure

    def neighbor_collision_free(self, new_sample, neighbors):
        ret = []
        for neighbor in neighbors:
            collision_free = self.map_info.collision_free(
                RouteInfo([neighbor]).append(TreeNode(new_sample))
            )
            ret.append(collision_free)
        return ret

    def choose_parent(self, new_sample, neighbors, collision_free_list):
        min_dist = np.inf
        sample_node = TreeNode(new_sample)
        for idx, neighbor in enumerate(neighbors):
            if not collision_free_list[idx]:
                continue
            dist = dist_calc(neighbor.coord, new_sample)
            if dist < min_dist:
                sample_node.parent = neighbor
        return sample_node.parent

    def rewire(
        self, sample_node: TreeNode, neighbors: List[TreeNode], collision_free_list
    ):
        for idx, neighbor in enumerate(neighbors):
            if not collision_free_list[idx]:
                continue
            dist = dist_calc(neighbor.coord, sample_node.coord)
            if sample_node.cost + dist < neighbor.cost:
                neighbor.parent = sample_node
                neighbor.cost = sample_node.cost + dist

    def get_route(self) -> RouteInfo:
        """the instance method to get route info

        Returns
        -------
        RouteInfo
            the route information containing the route from origin to target
        """
        return self.final_ret


if __name__ == "__main__":
    pass
