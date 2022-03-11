import pytest
from loguru import logger
from RRT.algorithm.basicRRT import BasicRRT
from RRT.algorithm.RRT_Connect import RRT_Connect
from RRT.algorithm.RRT_Star import RRT_Star
from RRT.algorithm.RRT_with_probability import RRT_With_Probability
from RRT.algorithm.RRT_star_with_bspline import RRT_Star_With_BSpline
from RRT.config import get_test_map
from RRT.core.info import MapInfo, MissionInfo
from RRT.util.visualize import visualize
from RRT.util.bspline import path_smooth_with_bspline


def test_basic_RRT():
    mission_info = MissionInfo(
        MapInfo(get_test_map())
    )
    alg: BasicRRT = BasicRRT(None, mission_info, 3)
    res = alg.run()
    logger.debug(f"nodes: {alg.search_tree.get_nodes()}")
    logger.debug(f"edges: {alg.search_tree.get_edges()}")
    logger.debug(f"algorithm running result: {res}")

    # route_info = alg.get_route()
    route_info = alg.get_route()
    logger.debug(f"path: {route_info.get_route()}")
    logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
    logger.debug(f"length: {route_info.get_length()}")

    visualize(mission_info, route_info, f'test_{alg.__class__}.png')

def test_RRT_with_probability():
    mission_info = MissionInfo(
        MapInfo(get_test_map())
    )
    alg: RRT_With_Probability = RRT_With_Probability(None, mission_info, 0.5, 3)
    res = alg.run()
    logger.debug(f"nodes: {alg.search_tree.get_nodes()}")
    logger.debug(f"edges: {alg.search_tree.get_edges()}")
    logger.debug(f"algorithm running result: {res}")

    # route_info = alg.get_route()
    route_info = alg.get_route()
    logger.debug(f"path: {route_info.get_route()}")
    logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
    logger.debug(f"length: {route_info.get_length()}")

    visualize(mission_info, route_info, f'test_{alg.__class__}.png')

def test_RRT_connect():
    mission_info = MissionInfo(
        MapInfo(get_test_map())
    )
    alg: RRT_Connect = RRT_Connect(None, mission_info, 0.5, 3)
    res = alg.run()
    logger.debug(f"nodes: {alg.ret_tree.get_nodes()}")
    logger.debug(f"edges: {alg.ret_tree.get_edges()}")
    logger.debug(f"algorithm running result: {res}")

    # route_info = alg.get_route()
    route_info = alg.get_route()
    logger.debug(f"path: {route_info.get_route()}")
    logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
    logger.debug(f"length: {route_info.get_length()}")

    visualize(mission_info, route_info, f'test_{alg.__class__}.png')


def test_RRT_star():
    mission_info = MissionInfo(
        MapInfo(get_test_map())
    )
    alg: RRT_Star = RRT_Star(None, mission_info, 0.5, 3, 10000)
    res = alg.run()
    logger.debug(f"nodes: {alg.ret_tree.get_nodes()}")
    logger.debug(f"edges: {alg.ret_tree.get_edges()}")
    logger.debug(f"algorithm running result: {res}")

    # route_info = alg.get_route()
    route_info = alg.get_route()
    logger.debug(f"path: {route_info.get_route()}")
    logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
    logger.debug(f"length: {route_info.get_length()}")

    logger.debug(alg.__class__)
    visualize(mission_info, route_info, f'test_{alg.__class__}.png')

def test_RRT_star_with_bspline():
    mission_info = MissionInfo(
        MapInfo(get_test_map())
    )
    alg: RRT_Star_With_BSpline = RRT_Star_With_BSpline(None, mission_info, 0.5, 3, 10000)
    res = alg.run()
    logger.debug(f"nodes: {alg.ret_tree.get_nodes()}")
    logger.debug(f"edges: {alg.ret_tree.get_edges()}")
    logger.debug(f"algorithm running result: {res}")

    # route_info = alg.get_route()
    route_info = alg.get_route()
    route_info._coords = path_smooth_with_bspline(route_info._coords)
    logger.debug(f"path: {route_info.get_route()}")
    logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
    logger.debug(f"length: {route_info.get_length()}")

    logger.debug(alg.__class__)
    visualize(mission_info, route_info, f'test_{alg.__class__}.png')
