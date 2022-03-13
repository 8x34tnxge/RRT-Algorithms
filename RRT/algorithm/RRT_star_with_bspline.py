import random

import numpy as np
from RRT.algorithm.RRT_template import RRT_Template
from RRT.core import RRT
from RRT.core.info import DroneInfo, MissionInfo, RouteInfo
from RRT.util.samplemethod import resample, random_sample
from RRT.util.distcalc import dist_calc
from loguru import logger
from RRT.core.sign import FAILURE
from RRT.util.extendmethod import extend_with_rewire

class RRT_Star_With_BSpline(RRT_Template):
    def __init__(
        self,
        drone_info: DroneInfo,
        mission_info: MissionInfo,
        explore_prob: np.float64,
        step_size: np.float64,
        max_attempts: np.int32 = np.Infinity,
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
            the maximum number of attempts, by default np.Infinity
        """
        super().__init__(
            drone_info, mission_info, explore_prob, step_size, max_attempts
        )

        # init forward-search & backward-search & finally merged tree
        origin = mission_info.origin
        target = mission_info.target

        self.forward_search_tree = RRT(origin, target)
        self.backward_search_tree = RRT(target, origin)
        self.ret_tree = RRT(origin, target)

        # [ ] only contain forward-search & backward-search
        self.tree_roots = [self.forward_search_tree, self.backward_search_tree]

        # [ ] add more attr(s)

    def run(self) -> bool:
        """the method to run the basic RRT algorithm

        Returns
        -------
        bool
            whether basic RRT algorithm reach the target from origin
        """
        attempt_cnt = 0
        origin = self.mission_info.origin
        target = self.mission_info.target
        # while not self.ret_tree.is_reach_target:
        for _ in range(self.max_attempts):
            attempt_cnt += 1
            curr_tree = self.tree_roots[0]

            explore = random.random() < self.explore_prob
            if explore:
                new_sample = random_sample(
                    self.map_info.min_border, self.map_info.max_border
                )
            else:
                new_sample = curr_tree.target

            neighbors = curr_tree.get_nearest_neighbors(new_sample, num=1)
            neighbor_info = curr_tree.get_node_attr(neighbors[0])["coord"]

            if explore:
                new_sample = resample(neighbor_info, new_sample, self.step_size)
            else:
                out_range = dist_calc(neighbor_info, new_sample) > self.step_size
                new_sample = (
                    resample(neighbor_info, new_sample, self.step_size)
                    if out_range
                    else new_sample
                )

            extend_with_rewire(self.map_info, curr_tree, curr_tree.origin, new_sample, 5, smooth_method='bspline')

            merged_tree = RRT.merge_from_trees(self.tree_roots, origin, target)
            # logger.debug(merged_tree.is_reach_target)
            # [ ] optimizing instand of direct breaking
            if merged_tree.is_reach_target:
                self.ret_tree = merged_tree
                break

            self.swap_tree()

            if np.isfinite(self.max_attempts) and (attempt_cnt // 2) > self.max_attempts:
                break

        if attempt_cnt > self.max_attempts:
            return FAILURE
        return self.ret_tree.is_reach_target

    def get_route(self) -> RouteInfo:
        """the instance method to get route info

        Returns
        -------
        RouteInfo
            the route information containing the route from origin to target
        """
        return self.ret_tree.get_route(
            origin=self.mission_info.origin, target=self.mission_info.target
        )

    def swap_tree(self):
        """the instance method to swap/reverse the tree root"""
        self.tree_roots.reverse()
