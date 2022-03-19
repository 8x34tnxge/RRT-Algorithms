from __future__ import annotations

import numpy as np
from typing import List
from RRT.algorithm.RRT_template import RRT_Template
from RRT.core.info import DroneInfo
from RRT.core.mission_info import MissionInfo
from RRT.core.route_info import RouteInfo
from RRT.core.sign import AlgStatus, Status
from RRT.core.tree import Tree, TreeNode
from RRT.util.distcalc import dist_calc
from RRT.util.path_cat import cat_path
from RRT.util.samplemethod import steer


class Connect_RRT_Star(RRT_Template):
    """basic Rapidly-exploring Random Tree algorithm with only one forward-search tree"""

    def __init__(
        self,
        drone_info: DroneInfo,
        mission_info: MissionInfo,
        explore_prob: np.float64,
        step_size: np.float64,
        neighbor_num: np.int32 = 5,
        max_attempts: np.int32 = np.inf,
    ):
        """the init method of RRT

        Parameters
        ----------
        drone_info : DroneInfo
            the drone infomation
        mission_info : MissionInfo
            the mission infomation
        step_size : np.float64
            the size/length of each step
        max_attempts : np.int32, optional
            the maximum number of attempts, by default np.inf
        """
        super().__init__(
            drone_info, mission_info, explore_prob, step_size, max_attempts
        )
        self.neighbor_num = neighbor_num
        self.forward_tree: Tree = Tree(mission_info.origin)
        self.backward_tree: Tree = Tree(mission_info.target)
        self.trees = [self.forward_tree, self.backward_tree]

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

            new_sample = self.sample(self.trees[-1].nodes[0].coord)
            neighbors, neighbor_dist = self.trees[0].get_nearest_neighbors(new_sample, self.neighbor_num)
            new_sample = steer(neighbors[np.argmin(neighbor_dist)].coord, new_sample, self.step_size)

            sample_node = TreeNode(new_sample)
            if (
                self.extend(self.trees[0], sample_node, neighbors)
                != AlgStatus.Trapped
            ):
                if self.connect(self.trees[1], sample_node) == AlgStatus.Reached:
                    ret = cat_path(
                        self.forward_tree.get_route(
                            self.forward_tree.get_node(sample_node)
                        ),
                        self.backward_tree.get_route(
                            self.backward_tree.get_node(sample_node)
                        ),
                    )
                    if self.final_ret is None:
                        self.final_ret = ret
                    elif ret.get_length() < self.final_ret.get_length():
                        self.final_ret = ret

            self.swap_tree()

        if self.final_ret is not None:
            return Status.Success
        return Status.Failure

    def extend(self, tree: Tree, sample_node, neighbors):
        collision_free_list = self.neighbor_collision_free(sample_node.coord, neighbors)
        parent = self.choose_parent(sample_node.coord, neighbors, collision_free_list)
        if parent is None:
            return AlgStatus.Trapped

        new_node = tree.add_node(sample_node.coord, parent)

        self.rewire(new_node, neighbors, collision_free_list)

        return AlgStatus.Advanced

    def connect(self, tree: Tree, sample_node):
        while True:
            neighbor_nodes, neighbor_dist = tree.get_nearest_neighbors(sample_node.coord, self.neighbor_num)
            new_sample = steer(
                neighbor_nodes[np.argmin(neighbor_dist)].coord, sample_node.coord, self.step_size
            )
            dist = dist_calc(sample_node.coord, new_sample)

            status = self.extend(tree, TreeNode(new_sample), neighbor_nodes)

            if status == AlgStatus.Trapped:
                return AlgStatus.Trapped
            if status != AlgStatus.Trapped and dist == 0:
                return AlgStatus.Reached

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

    def swap_tree(self):
        self.trees.reverse()


if __name__ == "__main__":
    pass
