from __future__ import annotations

import numpy as np
from RRT.core.info import DroneInfo
from RRT.core.mission_info import MissionInfo
from RRT.core.route_info import RouteInfo
from RRT.core.sign import Status
from RRT.core.tree import TreeNode
from RRT.util.distcalc import dist_calc

class A_Star:
    def __init__(
        self,
        drone_info: DroneInfo,
        mission_info: MissionInfo,
        *args, **kwargs
    ):
        self.drone_info = drone_info
        self.mission_info = mission_info
        pass

    def run(self) -> bool:
        origin = self.mission_info.extract_origin_info()
        origin_node = TreeNode(origin)
        origin_node.real_dist = 0

        target = self.mission_info.extract_target_info()
        target_node = TreeNode(target)

        map_info = self.mission_info.map_info
        if map_info.map.ndim == 2:
            change_dir = [
                np.array([-1, 0]),
                np.array([1, 0]),
                np.array([0, 1]),
                np.array([0, -1]),
                np.array([-1, -1]),
                np.array([-1, 1]),
                np.array([1, -1]),
                np.array([1, 1]),
            ]
        elif map_info.map.ndim == 3:
                change_dir = [
                    np.array([-1, 0, 0]),
                    np.array([1, 0, 0]),
                    np.array([0, 1, 0]),
                    np.array([0, -1, 0]),
                    np.array([0, 0, 1]),
                    np.array([0, 0, -1]),
                    np.array([0, -1, -1]),
                    np.array([-1, 0, -1]),
                    np.array([-1, -1, 0]),
                    np.array([0, 1, 1]),
                    np.array([1, 0, 1]),
                    np.array([1, 1, 0]),
                    np.array([0, -1, 1]),
                    np.array([-1, 0, 1]),
                    np.array([-1, 1, 0]),
                    np.array([0, 1, -1]),
                    np.array([1, 0, -1]),
                    np.array([1, -1, 0]),
                    np.array([-1, 1, 1]),
                    np.array([1, -1, 1]),
                    np.array([1, 1, -1]),
                    np.array([1, -1, -1]),
                    np.array([-1, 1, -1]),
                    np.array([-1, -1, 1]),
                    np.array([1, 1, 1]),
                    np.array([-1, -1, -1]),
                ]
        else:
            return Status.Failure

        used_coord = [tuple(origin)]
        candidates = [origin_node]
        currNode = candidates.pop()
        while currNode != target_node:
            for dir in change_dir:
                new_coord = currNode.coord + dir
                if any(new_coord < 0) == True:
                    continue
                if any(new_coord >= np.array(map_info.map.shape)) == True:
                    continue
                if not map_info.check_point_feasible(np.array([new_coord]), node_num=1, ndim=map_info.map.ndim):
                    continue
                if tuple(new_coord) in used_coord:
                    continue
                used_coord.append(tuple(new_coord))
                real_dist = currNode.real_dist + 1
                estimate_dist = dist_calc(new_coord, target)
                new_node = TreeNode(new_coord, real_dist + estimate_dist, currNode)
                new_node.real_dist = real_dist
                candidates.append(new_node)
            candidates.sort(key=lambda x: x.cost, reverse=True)
            currNode = candidates.pop()

        nodes = [currNode]
        length = currNode.cost
        while currNode.parent is not None:
            currNode = currNode.parent
            nodes.insert(0, currNode)
        self.final_ret = RouteInfo(nodes, length)

        return Status.Success

    def get_route(self) -> RouteInfo:
        """the instance method to get route info

        Returns
        -------
        RouteInfo
            the route information containing the route from origin to target
        """
        return self.final_ret
