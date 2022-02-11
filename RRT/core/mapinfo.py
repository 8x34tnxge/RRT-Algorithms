import math
from typing import Any, List

import numpy as np
from nptyping import NDArray
from RRT.core.routeinfo import RouteInfo


# [ ] initial method
# [ ] the method to check whether the route is feasible
class MapInfo:
    def __init__(self, map: List[str], sample_level: str):
        """the initial method for MapInfo

        Parameters
        ----------
        map : List[str]
            the array of map. 0 stands for empty, 1 stands for origin, 2 stands for target, 3 stands for wall.
        sample_level : str
            sample level for algorithm. Only to "discrete", "continues"

        Raises
        ------
        ValueError
            The variable [map] or [sample_level] is invalid.
        """
        self.map: NDArray[(Any, ...)]= np.array(map)
        self.min_border: NDArray[Any] = np.zeros(self.map.ndim, dtype=np.int32)
        self.max_border: NDArray[Any] = np.ones(self.map.ndim, dtype=np.int32) * self.map.shape
        self.sample_level: str = sample_level

        if not self.is_valid():
            raise ValueError("The variable [map] or [sample_level] must be invalid. Please check these variables!")

    def is_valid(self) -> bool:
        # if map is not a array or tensor but object
        if self.map.dtype == "O":
            return False

        # if sample level is invalid
        if self.sample_level != "discrete" or self.sample_level != "continues":
            return False

        return True

    # [ ] consider the safe distance between drone and wall
    def is_feasible(self, route_info: RouteInfo) -> bool:
        """the instance method to determine whether the route solution is feasible

        Parameters
        ----------
        route_info : RouteInfo
            the route information from algorithm

        Returns
        -------
        bool
            whether the route solution is feasible
        """
        coordination: NDArray[Any]= route_info.get_route(type='coord')
        node_num, ndim = coordination.shape

        for nodeID in range(node_num):
            coord_pair = coordination[nodeID]
            candidates = [set() for _ in range(ndim)]

            for dim in range(ndim):
                coord= coord_pair[dim]

                if coord== int(coord):
                    candidates[dim].add(coord)
                    if coord >= 1:
                        candidates[dim].add(coord - 1)
                else:
                    candidates.add(math.ceil(coord))
