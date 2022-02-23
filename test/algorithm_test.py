from RRT.config import get_test_map
from RRT.core.mapinfo import MapInfo
from RRT.core.missioninfo import MissionInfo
from RRT.algorithm.basicRRT import BasicRRT
from loguru import logger


def test_basic_RRT():
    mission_info = MissionInfo(
        MapInfo(get_test_map())
    )
    alg = BasicRRT(None, mission_info, 0.9, 1, 100)
    res = alg.run()
    logger.debug(f"algorithm running result: {res}")

    route_info = alg.get_route()
    logger.debug(f"path: {route_info.get_route()}")
    logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
    logger.debug(f"length: {route_info.get_length()}")

def test_RRT_connect():
    pass


def test_RRT_star():
    pass
