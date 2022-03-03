import math
from typing import Any, List
from loguru import logger

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

        self.min_border: NDArray[Any] = np.zeros(self.map.ndim, dtype=np.int32) - 0.5
        self.max_border: NDArray[Any] = (
            np.ones(self.map.ndim, dtype=np.int32) * self.map.shape
        ) - 0.5

        self.sample_level: str = sample_level

        if not self._is_valid():
            raise ValueError(
                "The variable [map] or [sample_level] must be invalid. Please check these variables!"
            )


    def _is_valid(self) -> bool:
        """the private method to check whether the given map info is valid

        Returns
        -------
        bool
            the result of whether the given map info is valid
        """
        # if map is not a array or tensor but object
        if self.map.dtype == "O":
            return Failure

        # if sample level is invalid
        if self.sample_level != "discrete" and self.sample_level != "continues":
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
        coordination: NDArray[Any] = route_info.get_route(route_type="coord")
        node_num, ndim = coordination.shape

        # coord-pair's dimension must be equal to map's
        assert ndim == self.map.ndim

        # complete the detailed route filing with line points
        new_coordination = np.linspace(
            coordination[0, :], coordination[1, :], num=fill_num
        )
        for node_id in range(2, node_num):
            new_coordination = np.concatenate(
                (
                    new_coordination,
                    np.linspace(
                        coordination[node_id-1, :], coordination[node_id, :], num=fill_num
                    ),
                ),
            )

        # check the feasibility
        return self.check_point_feasible(new_coordination, *new_coordination.shape)

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

                ceil_coord = math.ceil(coord)
                floor_coord = math.floor(coord)

                if coord - floor_coord > 0.5:
                    candidates[dim].add(ceil_coord)
                elif coord - floor_coord < 0.5:
                    candidates[dim].add(floor_coord)
                else:
                    candidates[dim].add(ceil_coord)
                    candidates[dim].add(floor_coord)

                # if coord == int(coord):
                #     # avoid the boundary
                #     if coord < self.map.shape[dim]:
                #         candidates[dim].add(coord)
                #     if coord > 1:
                #         candidates[dim].add(coord - 1)
                # else:
                #     candidates[dim].add(math.floor(coord))

            # check whether each possible position is wall (because even the boundary of wall is infeasible)
            combinations = combination_from_candidates(
                candidates, converter=lambda x: tuple(x)
            )
            for comb in combinations:
                if self.map[tuple(map(int, comb))] == WALL:
                    return Failure

        return Success
