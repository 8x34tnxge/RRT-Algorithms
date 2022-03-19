from RRT.core.route_info import RouteInfo
from RRT.core.tree_node import TreeNode


def cat_path(head_route_info: RouteInfo, tail_route_info: RouteInfo):
    coord = []
    coord.extend(head_route_info._coords)
    coord.extend(list(reversed(tail_route_info._coords))[1:])

    return RouteInfo(
        list(map(lambda x: TreeNode(x), coord)),
        head_route_info.get_length() + tail_route_info.get_length(),
    )
