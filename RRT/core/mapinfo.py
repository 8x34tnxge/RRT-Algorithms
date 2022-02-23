import math
from typing import Any, List

import numpy as np
from nptyping import NDArray
from RRT.core.routeinfo import RouteInfo
from RRT.core.sign import Failure, Success
from RRT.util.comb import combination_from_candidates

# declare map constant
EMPTY = 0
ORIGIN = 1
TARGET = 2
WALL = 3

# [x] initial method
# [x] the method to check whether the route is feasible
class MapInfo:
    def __init__(self, map: List[str], sample_level: str = "continues"):
        """the initial method for MapInfo

        Parameters
        ----------
        map : List[str]
            the array of map. 0 stands for empty, 1 stands for origin, 2 stands for target, 3 stands for wall.
        sample_level : str
            sample level for algorithm. Only to "discrete", "continues", by default 'continues'

        Raises
        ------
        ValueError
            The variable [map] or [sample_level] is invalid.
        """
        self.map: NDArray[(Any, ...)] = np.array(map)

        self.origin = self.extract_origin_info()
        self.target = self.extract_target_info()

        self.min_border: NDArray[Any] = np.zeros(self.map.ndim, dtype=np.int32)
        self.max_border: NDArray[Any] = (
            np.ones(self.map.ndim, dtype=np.int32) * self.map.shape
        )

        self.sample_level: str = sample_level

        if not self.is_valid():
            raise ValueError(
                "The variable [map] or [sample_level] must be invalid. Please check these variables!"
            )

    def extract_origin_info(self) -> NDArray[Any]:
        """the instance method to extract the origin infomation/coordination from the given map info

        Returns
        -------
        NDArray[Any]
            the origin infomation/coordination
        """
        x, y = np.nonzero(self.map == 1)
        # each map should have only one origin
        assert x.shape[0] == 1 and y.shape[0] == 1

        origin = np.array([x, y]).reshape([-1])
        return origin

    def extract_target_info(self) -> NDArray[Any]:
        """the instance method to extract the target infomation/coordination from the given map info

        Returns
        -------
        NDArray[Any]
            the target information/coordination
        """
        x, y = np.nonzero(self.map == 2)
        # each map should have only one target
        assert x.shape[0] == 1 and y.shape[0] == 1

        target = np.array([x, y]).reshape([-1])
        return target

    def is_valid(self) -> bool:
        """the instance method to check whether the given map info is valid

        Returns
        -------
        bool
            the result of whether the given map info is valid
        """
        # if map is not a array or tensor but object
        if self.map.dtype == "O":
            return Failure

        # if sample level is invalid
        if self.sample_level != "discrete" or self.sample_level != "continues":
            return Failure

        return Success

    # [ ] consider the safe distance between drone and wall
    def is_feasible(self, route_info: RouteInfo, fill_num: int = 30) -> bool:
        """the instance method to determine whether the route solution is feasible

        Parameters
        ----------
        route_info : RouteInfo
            the route information from algorithm
        fill_num : int
            the number of line points to be filled in route

        Returns
        -------
        bool
            whether the route solution is feasible
        """
        coordination: NDArray[Any] = route_info.get_route(type="coord")
        node_num, ndim = coordination.shape

        # coord-pair's dimension must be equal to map's
        assert ndim == self.map.ndim

        # complete the detailed route filing with line points
        new_coordination = np.linspace(
            (coordination[0, :], coordination[1, :]), num=fill_num
        )
        for node_id in range(2, node_num):
            np.concatenate(
                (
                    new_coordination,
                    np.linspace(
                        (coordination[node_id, :], coordination[node_id + 1, :])
                    ),
                )
            )

        # check the feasibility
        if (
            self.check_point_feasible(new_coordination, *new_coordination.shape)
            == Failure
        ):
            return Failure
        return True

    def check_point_feasible(
        self, coordination: NDArray[Any], node_num: int, ndim: int
    ) -> bool:
        """the instance method to check whether the point is feasible

        Parameters
        ----------
        coordination : NDArray[Any]
            the route coordination from route info
        node_num : int
            the number of node of the given route from route info
        ndim : int
            the number of dimension

        Returns
        -------
        bool
            whether each point in route is feasible
        """
        for nodeID in range(node_num):
            coord_pair = coordination[nodeID]
            candidates = [set() for _ in range(ndim)]

            for dim in range(ndim):
                coord = coord_pair[dim]

                if coord == int(coord):
                    # abandon the the boundary exceed situation
                    if coord < self.map.shape[dim]:
                        candidates[dim].add(coord)
                    if coord >= 1:
                        candidates[dim].add(coord - 1)
                else:
                    candidates.add(math.ceil(coord))

            # check whether each possible position is wall (because even the boundary of wall is infeasible)
            combinations = combination_from_candidates(
                candidates, converter=lambda x: tuple(x)
            )
            for comb in combinations:
                if self.map[comb] == WALL:
                    return Failure

        return Success
