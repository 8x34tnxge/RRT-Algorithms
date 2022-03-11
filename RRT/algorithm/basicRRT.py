from __future__ import annotations

import random

import numpy as np
from RRT.algorithm.RRT_template import RRT_Template
from RRT.core import RRT
from RRT.core.info import DroneInfo, MissionInfo, RouteInfo
from RRT.core.sign import FAILURE
from RRT.util.distcalc import dist_calc
from RRT.util.extendmethod import directly_extend
from RRT.util.samplemethod import random_sample, resample


class BasicRRT(RRT_Template):
    """basic Rapidly-exploring Random Tree algorithm with only one forward-search tree"""

    def __init__(
        self,
        drone_info: DroneInfo,
        mission_info: MissionInfo,
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
        step_size : np.float64
            the size/length of each step
        max_attempts : np.int32, optional
            the maximum number of attempts, by default np.Infinity
        """
        super().__init__(
            drone_info, mission_info, 1, step_size, max_attempts
        )
        self.search_tree: RRT = RRT(mission_info.origin, mission_info.target)

    def run(self) -> bool:
        """the method to run the basic RRT algorithm

        Returns
        -------
        bool
            whether basic RRT algorithm reach the target from origin
        """
        attempt_cnt = 0
        while not self.search_tree.is_reach_target:
            attempt_cnt += 1

            new_sample = random_sample(
                self.map_info.min_border, self.map_info.max_border
            )

            neighbors = self.search_tree.get_nearest_neighbors(new_sample, num=1)
            neighbor_info = self.search_tree.get_node_attr(neighbors[0])["coord"]

            new_sample = resample(neighbor_info, new_sample, self.step_size)

            if self.mission_info.map_info.is_feasible(
                self.search_tree.get_route(target=neighbor_info).append(new_sample)
            ):
                directly_extend(self.search_tree, new_sample, neighbors[0])

            self.search_tree.update_status()
            if np.isfinite(self.max_attempts) and attempt_cnt > self.max_attempts:
                break

        if attempt_cnt > self.max_attempts:
            return FAILURE
        return self.search_tree.is_reach_target

    def get_route(self) -> RouteInfo:
        """the instance method to get route info

        Returns
        -------
        RouteInfo
            the route information containing the route from origin to target
        """
        return self.search_tree.get_route(
            origin=self.mission_info.origin, target=self.mission_info.target
        )
