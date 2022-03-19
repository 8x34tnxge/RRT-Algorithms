from typing import Any

import numpy as np
from nptyping import NDArray
from RRT.core import RRT as RRT
from RRT.core.info import MapInfo
from RRT.util.distcalc import dist_calc


def directly_extend(tree: RRT, new_node_info: NDArray[Any], nearest_node_ID: int):
    """the method to directly extend the tree from nearest node using new node

    Parameters
    ----------
    tree : RRT
        the tree storing the node and edge info
    new_node_info : NDArray[Any]
        the info of node to be inserted to tree
    nearest_node_ID : int
        the id of the nearest node from new node
    """
    new_node_ID = tree.add_node(new_node_info)
    tree.add_edge(nearest_node_ID, new_node_ID)
    return new_node_ID


def extend_with_rewire(
    map_info: MapInfo,
    tree: RRT,
    root: NDArray[Any],
    new_node_info: NDArray[Any],
    radius: float,
    smooth_method="line",
) -> RRT:
    neighbors = tree.get_nearest_neighbors(
        new_node_info, radius, condition=lambda x: x <= radius
    )

    # if self.mission_info.map_info.is_feasible(
    #     curr_tree.get_route(target=neighbor_info).append(new_sample)
    # ):
    # insert new node
    min_val, min_id = np.Infinity, -1
    for neighbor in neighbors:
        neighbor_info = tree.get_node_attr(neighbor)["coord"]
        route_info = tree.get_route(root, neighbor_info)
        length = route_info.get_length() + dist_calc(neighbor_info, new_node_info)
        if length < min_val and map_info.is_feasible(
            tree.get_route(target=neighbor_info).append(new_node_info),
            method=smooth_method,
        ):
            min_val = length
            min_id = neighbor
    if min_id == -1:
        return
    new_node_id = tree.add_node(new_node_info)
    tree.add_edge(min_id, new_node_id)
    tree.update_status()

    # rewire edge
    for neighbor in neighbors:
        neighbor_info = tree.get_node_attr(neighbor)["coord"]

        origin_route_info = tree.get_route(root, neighbor_info)
        route_info = tree.get_route(root, new_node_info)

        origin_length = origin_route_info.get_length()
        length = route_info.get_length() + dist_calc(neighbor_info, new_node_info)
        if length < origin_length:
            # adjacent_node_id = []
            # adjacent_node_id.extend(tree.tree.adj[neighbor])
            # adjacent_node_id = tree.tree.adj[neighbor]
            # for node_id in adjacent_node_id:
            #     tree.tree.remove_edge(neighbor, node_id)
            tree.add_edge(new_node_id, neighbor)
