from __future__ import annotations

import numpy as np
from RRT.algorithm.RRT_template import RRT_Template
from RRT.core.info import DroneInfo
from RRT.core.mission_info import MissionInfo
from RRT.core.route_info import RouteInfo
from RRT.core.sign import Status
from RRT.core.tree import Tree, TreeNode
from RRT.util.distcalc import dist_calc
from RRT.util.samplemethod import steer


class RRT_With_Probability(RRT_Template):
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
            neighbors, neighbor_dist = self.search_tree.get_nearest_neighbors(new_sample)
            new_sample = steer(neighbors[np.argmin(neighbor_dist)].coord, new_sample, self.step_size)

            if self.map_info.collision_free(
                RouteInfo([neighbors[0]]).append(TreeNode(new_sample))
            ):
                new_node = self.search_tree.add_node(new_sample, neighbors[0])

                if dist_calc(new_sample, self.mission_info.target) > self.step_size:
                    continue
                if not self.map_info.collision_free(
                    RouteInfo([new_node]).append(TreeNode(self.mission_info.target))
                ):
                    continue

                target_node = self.search_tree.add_node(
                    self.mission_info.target, new_node
                )
                self.final_ret = self.search_tree.get_route(target_node)

        if self.final_ret is not None:
            return Status.Success
        return Status.Failure

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
