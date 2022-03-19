import random
from abc import ABC, abstractmethod
from typing import Union

import numpy as np
from RRT.core.info import DroneInfo
from RRT.core.map_space import MapSpace
from RRT.core.mission_info import MissionInfo
from RRT.core.route_info import RouteInfo
from RRT.util.samplemethod import random_sample


class RRT_Template(ABC):
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
        self.drone_info: DroneInfo = drone_info
        self.mission_info: MissionInfo = mission_info
        self.map_info: MapSpace = mission_info.map_info
        self.explore_prob: np.float64 = explore_prob
        self.max_attempts: Union[np.int32, None] = max_attempts
        self.step_size: np.float64 = step_size

        self.final_ret: RouteInfo = None

    @abstractmethod
    def run(self) -> bool:
        """the method to run the basic RRT algorithm

        Returns
        -------
        bool
            whether basic RRT algorithm reach the target from origin
        """
        pass

    def sample(self, target=None):
        if random.random() < self.explore_prob:
            new_sample = random_sample(
                self.map_info.min_border, self.map_info.max_border
            )
        else:
            new_sample = self.mission_info.target if target is None else target

        return new_sample

    @abstractmethod
    def get_route(self) -> RouteInfo:
        """the instance method to get route info

        Returns
        -------
        RouteInfo
            the route information containing the route from origin to target
        """
        pass
