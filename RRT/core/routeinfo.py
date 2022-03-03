from typing import Any, List, Union
from loguru import logger

import numpy as np
from nptyping import NDArray
from RRT.util.distcalc import dist_calc


class RouteInfo:
    def __init__(
        self,
        sequence: NDArray[(Any)],
        coords: NDArray[(Any, Any)],
        length: np.float64,
    ):
        """the initial method for RouteInfo

        Parameters
        ----------
        sequence : List[Any]
            the list of point/node ID in order.
        coords : List[Any]
            the multi-dim list of coordination of points/nodes respectively.
        length : np.float64
            the length of the route
        """
        self._sequence: List[Any] = sequence
        self._coords: List[Any]= coords
        self._length: np.float64 = length

    def append(self, node_coord):
        self._sequence.append(0)
        self._length += dist_calc(self._coords[-1], node_coord)
        self._coords.append(node_coord)
        return self

    def get_route(
        self, route_type: str = "sequence"
    ) -> Union[NDArray[(Any)], NDArray[(Any, Any)]]:
        """the instance method to obtain the needed route

        Parameters
        -------
        route_type : str
            the type of needed route, only be 'sequence' or 'coord'

        Returns
        -------
        Union[NDArray[(Any)], NDArray[(Any, Any)]]
            the needed route information

        Raises
        ------
        ValueError
            the variable [route_type] is not 'sequence' or 'coord'
        """
        if route_type == "sequence":
            return np.array(self._sequence)
        elif route_type == "coord":
            return np.array(self._coords)
        else:
            raise ValueError(
                f"The variable [route_type] must be 'sequence' or 'coord'! its value is [{route_type}] now."
            )

    def get_length(self) -> np.float64:
        """the instance method to get the length of the route

        Returns
        -------
        np.float64
            the length of the route
        """
        return self._length

    def is_feasible(self) -> bool:
        """the method to get the result of whether the current solution is feasible

        Returns
        -------
        bool
            the result of whether the current solution is feasible
        """
        if self._sequence is None or self._coords is None or self._length is None:
            return False
        else:
            return True
