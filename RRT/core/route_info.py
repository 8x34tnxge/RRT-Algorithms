from typing import Any, List, Union

import numpy as np
from nptyping import NDArray
from RRT.util.distcalc import dist_calc
from RRT.core.tree_node import TreeNode


class RouteInfo:
    def __init__(
        self,
        nodes: List[TreeNode],
        length: np.float64 = 0,
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
        self._coords: List[Any] = list(map(lambda node: node.coord, nodes))
        self._length: np.float64 = length

    def append(self, node: TreeNode):
        self._length += dist_calc(self._coords[-1], node.coord)
        self._coords.append(node.coord)

        return self

    def get_route(self) -> Union[NDArray[(Any)], NDArray[(Any, Any)]]:
        """the instance method to obtain the needed route

        Returns
        -------
        Union[NDArray[(Any)], NDArray[(Any, Any)]]
            the needed route information

        """
        return np.array(self._coords)

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
        if self._coords is None or self._length is None:
            return False
        else:
            return True

    def smooth_route(self, smooth_method, *args, **kwargs):
        self._coords = smooth_method(self._coords, *args, **kwargs)
        self.update_length()

    def update_length(self):
        if isinstance(self._coords, list):
            coords = np.array(self._coords)
        else:
            coords = self._coords
        point_num = coords.shape[0]
        length = 0
        for i in range(point_num - 1):
            length += dist_calc(coords[i, :], coords[i + 1, :])
        self.length = length
