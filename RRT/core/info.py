import math
import os
import pickle
from typing import Any, List, Union

from loguru import logger
import networkx as nx
import numpy as np
from nptyping import NDArray
from RRT.core.sign import EMPTY, FAILURE, ORIGIN, SUCCESS, TARGET, WALL
from RRT.util.bspline import path_smooth_with_bspline
from RRT.util.comb import combination_from_candidates
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
        self._coords: List[Any] = coords
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

    def save(self, save_name: str, save_dir: str):
        if not os.path.exists(save_dir):
            os.mkdir(save_dir)
            # raise ValueError("save directory is not exists")
        with open(os.path.join(save_dir, save_name + ".pickle"), "wb") as f:
            pickle.dump(self, f)

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


class DroneInfo:
    pass


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
            return FAILURE

        # if sample level is invalid
        if self.sample_level != "discrete" and self.sample_level != "continues":
            return FAILURE

        return SUCCESS

    # [ ] consider the safe distance between drone and wall
    def is_feasible(
        self, route_info: RouteInfo, fill_num: int = 30, method="line"
    ) -> bool:
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

        for coord in coordination:
            for dim in range(ndim):
                if coord[dim] < self.min_border[dim]:
                    return FAILURE
                if coord[dim] > self.max_border[dim]:
                    return FAILURE

        # complete the detailed route filing with line points
        if method == "line":
            new_coordination = np.linspace(
                coordination[0, :], coordination[1, :], num=fill_num
            )
            for node_id in range(2, node_num):
                new_coordination = np.concatenate(
                    (
                        new_coordination,
                        np.linspace(
                            coordination[node_id - 1, :],
                            coordination[node_id, :],
                            num=fill_num,
                        ),
                    ),
                )
        else:
            new_coordination = path_smooth_with_bspline(coordination)

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
                    return FAILURE

        return SUCCESS


class MissionInfo:
    def __init__(self, map_info: MapInfo):
        self.map_info = map_info

        self.origin = self.extract_origin_info()
        self.target = self.extract_target_info()

    def extract_origin_info(self) -> NDArray[Any]:
        """the instance method to extract the origin infomation/coordination from the given map info

        Returns
        -------
        NDArray[Any]
            the origin infomation/coordination
        """
        coord = np.nonzero(self.map_info.map == 1)
        origin = np.array(coord).reshape([-1])
        return origin

    def extract_target_info(self) -> NDArray[Any]:
        """the instance method to extract the target infomation/coordination from the given map info

        Returns
        -------
        NDArray[Any]
            the target information/coordination
        """
        coord = np.nonzero(self.map_info.map == 2)
        target = np.array(coord).reshape([-1])
        return target


class DistInfo:
    def __init__(self, tree: nx.classes.graph.Graph):
        self.route_mat = dict(nx.algorithms.all_pairs_shortest_path(tree))
        self.dist_mat = dict(nx.algorithms.all_pairs_shortest_path_length(tree))

    def update(self, tree: nx.classes.graph.Graph):
        self.route_mat = dict(nx.algorithms.all_pairs_shortest_path(tree))
        self.dist_mat = dict(nx.algorithms.all_pairs_shortest_path_length(tree))

    def has_path(self, tree, origin_id, target_id) -> bool:
        return nx.algorithms.has_path(tree, origin_id, target_id)

    def get_path(self, origin_id, target_id) -> List:
        return self.route_mat[origin_id][target_id]

    def get_path_length(self, origin_id, target_id) -> List:
        return self.dist_mat[origin_id][target_id]