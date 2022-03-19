from __future__ import annotations

import numpy as np
from RRT.algorithm.RRT_template import RRT_Template
from RRT.core.info import DroneInfo
from RRT.core.mission_info import MissionInfo
from RRT.core.route_info import RouteInfo
from RRT.core.sign import AlgStatus, Status
from RRT.core.tree import Tree, TreeNode
from RRT.util.distcalc import dist_calc
from RRT.util.path_cat import cat_path
from RRT.util.samplemethod import steer


class RRT_Connect(RRT_Template):
    """basic Rapidly-exploring Random Tree algorithm with only one forward-search tree"""

    def __init__(
        self,
        drone_info: DroneInfo,
        mission_info: MissionInfo,
        explore_prob: np.float64,
        step_size: np.float64,
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
            neighbors, neighbor_dist = self.trees[0].get_nearest_neighbors(new_sample)
            new_sample = steer(neighbors[np.argmin(neighbor_dist)].coord, new_sample, self.step_size)

            sample_node = TreeNode(new_sample)
            if (
                self.extend(self.trees[0], sample_node, neighbors[0])
                != AlgStatus.Trapped
            ):
                flag = self.connect(self.trees[1], sample_node)
                if flag == AlgStatus.Reached:
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

    def extend(self, tree: Tree, sample_node, neighbor_node):
        if self.map_info.collision_free(RouteInfo([neighbor_node]).append(sample_node)):
            tree.add_node(sample_node.coord, neighbor_node)
            return AlgStatus.Advanced
        return AlgStatus.Trapped

    def connect(self, tree: Tree, sample_node):
        while True:
            neighbor_nodes, neighbor_dist = tree.get_nearest_neighbors(sample_node.coord)
            new_sample = steer(
                neighbor_nodes[np.argmin(neighbor_dist)].coord, sample_node.coord, self.step_size
            )
            dist = dist_calc(sample_node.coord, new_sample)

            status = self.extend(tree, TreeNode(new_sample), neighbor_nodes[0])

            if status == AlgStatus.Trapped:
                return AlgStatus.Trapped
            if status != AlgStatus.Trapped and dist == 0:
                return AlgStatus.Reached

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
