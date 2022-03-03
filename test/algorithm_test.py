from RRT.config import get_test_map
from RRT.core.mapinfo import MapInfo
from RRT.core.missioninfo import MissionInfo
from RRT.algorithm.basicRRT import BasicRRT
from loguru import logger
from RRT.util.visualize import visualize


def test_basic_RRT():
    mission_info = MissionInfo(
        MapInfo(get_test_map())
    )
    alg: BasicRRT = BasicRRT(None, mission_info, 0.5, 3)
    res = alg.run()
    logger.debug(f"nodes: {alg.search_tree.get_nodes()}")
    logger.debug(f"edges: {alg.search_tree.get_edges()}")
    logger.debug(f"algorithm running result: {res}")

    neighbors = alg.search_tree.get_nearest_neighbors(alg.mission_info.target, num=1)
    neighbor_info = alg.search_tree.get_nodes()[neighbors[0]]["coord"]
    # route_info = alg.get_route()
    route_info = alg.search_tree.get_route(target=neighbor_info)
    logger.debug(f"path: {route_info.get_route()}")
    logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
    logger.debug(f"length: {route_info.get_length()}")

    visualize(mission_info, route_info, 'test.png')

def test_RRT_connect():
    pass


def test_RRT_star():
    pass
