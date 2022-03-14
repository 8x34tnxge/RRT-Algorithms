from abc import ABC, abstractmethod
from typing import Any, Union

import numpy as np
from nptyping import NDArray
from RRT.core.info import DroneInfo, MapInfo, MissionInfo, RouteInfo


class RRT_Template(ABC):
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
        self.drone_info: DroneInfo = drone_info
        self.mission_info: MissionInfo = mission_info
        self.map_info: MapInfo = mission_info.map_info
        self.explore_prob: np.float64 = explore_prob
        self.max_attempts: Union[np.int32, None] = max_attempts
        self.step_size: np.float64 = step_size

        # initialize the forward search tree
        origin: NDArray[Any]
        target: NDArray[Any]
        origin, target = mission_info.origin, mission_info.target

    @abstractmethod
    def run(self) -> bool:
        """the method to run the basic RRT algorithm

        Returns
        -------
        bool
            whether basic RRT algorithm reach the target from origin
        """
        pass

    @abstractmethod
    def get_route(self) -> RouteInfo:
        """the instance method to get route info

        Returns
        -------
        RouteInfo
            the route information containing the route from origin to target
        """
        pass
