from loguru import logger
from RRT.algorithm.basicRRT import BasicRRT
from RRT.algorithm.a_star import A_Star
from RRT.algorithm.RRT_with_probability import RRT_With_Probability
from RRT.algorithm.RRT_connect import RRT_Connect
from RRT.algorithm.RRT_star import RRT_Star
from RRT.config import map_loader
from RRT.core.mission_info import MissionInfo
from RRT.core.map_space import MapSpace
from RRT.util.visualize import visualize
from RRT.core.sign import Status

def test_basic_A_Star_2d():
    mission_info = MissionInfo(MapSpace(map_loader.get_map("test_1")))
    alg: A_Star = A_Star (None, mission_info, 3, 3000)
    res = alg.run()
    if res == Status.Success:
        logger.debug("success")
        route_info = alg.get_route()
        logger.debug(f"coordination: {route_info.get_route()}")
        logger.debug(f"length: {route_info.get_length()}")
        fig = visualize(mission_info, route_info)
        fig.savefig("./test_1.png")

def test_basic_RRT_2d():
    mission_info = MissionInfo(MapSpace(map_loader.get_map("test_1")))
    alg: BasicRRT = BasicRRT(None, mission_info, 3, 3000)
    res = alg.run()
    logger.debug(len(alg.search_tree.nodes))
    if res == Status.Success:
        logger.debug("success")
        route_info = alg.get_route()
        logger.debug(f"coordination: {route_info.get_route()}")
        logger.debug(f"length: {route_info.get_length()}")
        fig = visualize(mission_info, route_info)
        fig.savefig("./test_1.png")


def test_basic_RRT_with_probability_2d():
    mission_info = MissionInfo(MapSpace(map_loader.get_map("test_1")))
    alg: RRT_With_Probability = RRT_With_Probability(None, mission_info, 0.7, 3, 3000)
    res = alg.run()
    logger.debug(len(alg.search_tree.nodes))
    if res == Status.Success:
        logger.debug("success")
        route_info = alg.get_route()
        # logger.debug(f"nodes: {alg.search_tree.get_nodes()}")
        # logger.debug(f"edges: {alg.search_tree.get_edges()}")
        # logger.debug(f"algorithm running result: {res}")

        # # route_info = alg.get_route()
        # route_info = alg.get_route()
        # logger.debug(f"path: {route_info.get_route()}")
        logger.debug(f"coordination: {route_info.get_route()}")
        logger.debug(f"length: {route_info.get_length()}")
        fig = visualize(mission_info, route_info)
        fig.savefig("./test_2.png")

    alg_name = alg.__class__.__name__
    # visualize(mission_info, route_info)


def test_RRT_connect_2d():
    mission_info = MissionInfo(MapSpace(map_loader.get_map("test_1")))
    alg: RRT_Connect = RRT_Connect(None, mission_info, 0.7, 3, 3000)
    res = alg.run()
    if res == Status.Success:
        route_info = alg.get_route()
        logger.debug(f"coordination: {route_info.get_route()}")
        logger.debug(f"length: {route_info.get_length()}")
        fig = visualize(mission_info, route_info)
        fig.savefig("./test_3.png")


def test_basic_RRT_star_2d():
    mission_info = MissionInfo(MapSpace(map_loader.get_map("test_1")))
    alg: RRT_Star = RRT_Star(None, mission_info, 0.7, 3, 5, 3000)
    res = alg.run()
    logger.debug(len(alg.search_tree.nodes))
    if res == Status.Success:
        logger.debug("success")
        route_info = alg.get_route()
        # logger.debug(f"nodes: {alg.search_tree.get_nodes()}")
        # logger.debug(f"edges: {alg.search_tree.get_edges()}")
        # logger.debug(f"algorithm running result: {res}")

        # # route_info = alg.get_route()
        # route_info = alg.get_route()
        # logger.debug(f"path: {route_info.get_route()}")
        logger.debug(f"coordination: {route_info.get_route()}")
        logger.debug(f"length: {route_info.get_length()}")
        fig = visualize(mission_info, route_info)
        fig.savefig("./test_4.png")

    # logger.debug(f"nodes: {alg.ret_tree.get_nodes()}")
    # logger.debug(f"edges: {alg.ret_tree.get_edges()}")
    # logger.debug(f"algorithm running result: {res}")

    # # route_info = alg.get_route()
    # route_info = alg.get_route()
    # logger.debug(f"path: {route_info.get_route()}")
    # logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
    # logger.debug(f"length: {route_info.get_length()}")

    # alg_name = alg.__class__.__name__
    # visualize(mission_info, route_info)


# def test_basic_RRT_2d():
#     mission_info = MissionInfo(MapInfo(map_loader.get_map("simple-map")))
#     alg: BasicRRT = BasicRRT(None, mission_info, 3)
#     res = alg.run()
#     logger.debug(f"nodes: {alg.search_tree.get_nodes()}")
#     logger.debug(f"edges: {alg.search_tree.get_edges()}")
#     logger.debug(f"algorithm running result: {res}")

#     # route_info = alg.get_route()
#     route_info = alg.get_route()
#     logger.debug(f"path: {route_info.get_route()}")
#     logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
#     logger.debug(f"length: {route_info.get_length()}")

#     alg_name = alg.__class__.__name__
#     visualize(mission_info, route_info)


# def test_RRT_with_probability_2d():
#     mission_info = MissionInfo(MapInfo(map_loader.get_map("simple-map")))
#     alg: RRT_With_Probability = RRT_With_Probability(None, mission_info, 0.5, 3)
#     res = alg.run()
#     logger.debug(f"nodes: {alg.search_tree.get_nodes()}")
#     logger.debug(f"edges: {alg.search_tree.get_edges()}")
#     logger.debug(f"algorithm running result: {res}")

#     # route_info = alg.get_route()
#     route_info = alg.get_route()
#     logger.debug(f"path: {route_info.get_route()}")
#     logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
#     logger.debug(f"length: {route_info.get_length()}")

#     alg_name = alg.__class__.__name__
#     visualize(mission_info, route_info)


# def test_RRT_connect_2d():
#     mission_info = MissionInfo(MapInfo(map_loader.get_map("simple-map")))
#     alg: RRT_Connect = RRT_Connect(None, mission_info, 0.5, 3)
#     res = alg.run()
#     logger.debug(f"nodes: {alg.ret_tree.get_nodes()}")
#     logger.debug(f"edges: {alg.ret_tree.get_edges()}")
#     logger.debug(f"algorithm running result: {res}")

#     # route_info = alg.get_route()
#     route_info = alg.get_route()
#     logger.debug(f"path: {route_info.get_route()}")
#     logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
#     logger.debug(f"length: {route_info.get_length()}")

#     alg_name = alg.__class__.__name__
#     visualize(mission_info, route_info)


# def test_RRT_star_2d():
#     mission_info = MissionInfo(MapInfo(map_loader.get_map("simple-map")))
#     alg: RRT_Star = RRT_Star(None, mission_info, 0.5, 3)
#     res = alg.run()
#     logger.debug(f"nodes: {alg.ret_tree.get_nodes()}")
#     logger.debug(f"edges: {alg.ret_tree.get_edges()}")
#     logger.debug(f"algorithm running result: {res}")

#     # route_info = alg.get_route()
#     route_info = alg.get_route()
#     logger.debug(f"path: {route_info.get_route()}")
#     logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
#     logger.debug(f"length: {route_info.get_length()}")

#     alg_name = alg.__class__.__name__
#     visualize(mission_info, route_info)


# def test_RRT_star_with_bspline_2d():
#     mission_info = MissionInfo(MapInfo(map_loader.get_map("simple-map")))
#     alg: RRT_Star_With_BSpline = RRT_Star_With_BSpline(
#         None, mission_info, 0.5, 3
#     )
#     res = alg.run()
#     logger.debug(f"nodes: {alg.ret_tree.get_nodes()}")
#     logger.debug(f"edges: {alg.ret_tree.get_edges()}")
#     logger.debug(f"algorithm running result: {res}")

#     # route_info = alg.get_route()
#     route_info = alg.get_route()
#     route_info.smooth_route(path_smooth_with_bspline)
#     logger.debug(f"path: {route_info.get_route()}")
#     logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
#     logger.debug(f"length: {route_info.get_length()}")

#     alg_name = alg.__class__.__name__
#     visualize(mission_info, route_info)


# def test_basic_RRT_3d():
#     mission_info = MissionInfo(MapInfo(map_loader.get_map("map")))
#     alg: BasicRRT = BasicRRT(None, mission_info, 3)
#     res = alg.run()
#     logger.debug(f"nodes: {alg.search_tree.get_nodes()}")
#     logger.debug(f"edges: {alg.search_tree.get_edges()}")
#     logger.debug(f"algorithm running result: {res}")

#     # route_info = alg.get_route()
#     route_info = alg.get_route()
#     logger.debug(f"path: {route_info.get_route()}")
#     logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
#     logger.debug(f"length: {route_info.get_length()}")

#     alg_name = alg.__class__.__name__
#     visualize(mission_info, route_info)


# def test_RRT_with_probability_3d():
#     mission_info = MissionInfo(MapInfo(map_loader.get_map("map")))
#     alg: RRT_With_Probability = RRT_With_Probability(None, mission_info, 0.5, 3)
#     res = alg.run()
#     logger.debug(f"nodes: {alg.search_tree.get_nodes()}")
#     logger.debug(f"edges: {alg.search_tree.get_edges()}")
#     logger.debug(f"algorithm running result: {res}")

#     # route_info = alg.get_route()
#     route_info = alg.get_route()
#     logger.debug(f"path: {route_info.get_route()}")
#     logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
#     logger.debug(f"length: {route_info.get_length()}")

#     alg_name = alg.__class__.__name__
#     visualize(mission_info, route_info)


# def test_RRT_connect_3d():
#     mission_info = MissionInfo(MapInfo(map_loader.get_map("map")))
#     alg: RRT_Connect = RRT_Connect(None, mission_info, 0.5, 3)
#     res = alg.run()
#     logger.debug(f"nodes: {alg.ret_tree.get_nodes()}")
#     logger.debug(f"edges: {alg.ret_tree.get_edges()}")
#     logger.debug(f"algorithm running result: {res}")

#     # route_info = alg.get_route()
#     route_info = alg.get_route()
#     logger.debug(f"path: {route_info.get_route()}")
#     logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
#     logger.debug(f"length: {route_info.get_length()}")

#     alg_name = alg.__class__.__name__
#     visualize(mission_info, route_info)


# def test_RRT_star_3d():
#     mission_info = MissionInfo(MapInfo(map_loader.get_map("map")))
#     alg: RRT_Star = RRT_Star(None, mission_info, 0.5, 3)
#     res = alg.run()
#     logger.debug(f"nodes: {alg.ret_tree.get_nodes()}")
#     logger.debug(f"edges: {alg.ret_tree.get_edges()}")
#     logger.debug(f"algorithm running result: {res}")

#     # route_info = alg.get_route()
#     route_info = alg.get_route()
#     logger.debug(f"path: {route_info.get_route()}")
#     logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
#     logger.debug(f"length: {route_info.get_length()}")

#     alg_name = alg.__class__.__name__
#     visualize(mission_info, route_info)


# def test_RRT_star_with_bspline_3d():
#     mission_info = MissionInfo(MapInfo(map_loader.get_map("map")))
#     alg: RRT_Star_With_BSpline = RRT_Star_With_BSpline(
#         None, mission_info, 0.5, 3
#     )
#     res = alg.run()
#     logger.debug(f"nodes: {alg.ret_tree.get_nodes()}")
#     logger.debug(f"edges: {alg.ret_tree.get_edges()}")
#     logger.debug(f"algorithm running result: {res}")

#     # route_info = alg.get_route()
#     route_info = alg.get_route()
#     route_info.smooth_route(path_smooth_with_bspline)
#     logger.debug(f"path: {route_info.get_route()}")
#     logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
#     logger.debug(f"length: {route_info.get_length()}")

#     alg_name = alg.__class__.__name__
#     visualize(mission_info, route_info)
