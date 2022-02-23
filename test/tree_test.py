import numpy as np
from RRT.core import RRT
from RRT.algorithm import BasicRRT
from loguru import logger

logger.remove(0)
logger.add("test.log")

test_origin = np.array([1])
test_target = np.array([1])

test_nodes = [np.array([x]) for x in range(10)]


def test_RRT_create():
    tmp = RRT(test_origin, test_target)
    assert isinstance(tmp, RRT)

def test_node():
    tmp = RRT(test_origin, test_target)
    for n in range(3):
        tmp.add_node(test_nodes[n])
    logger.debug(f"edges: {tmp.get_nodes()}")

def test_edge():
    tmp = RRT(test_origin, test_target)
    for n in range(3):
        tmp.add_node(test_nodes[n])
    tmp.add_edge(1, 2)
    tmp.add_edge(1, 3)

    logger.debug(f"edges: {tmp.get_edges()}")

def test_merge():
    t1 = RRT(test_origin, test_target)
    t2 = RRT(test_origin, test_target)

    for n in range(3):
        t1.add_node(test_nodes[n])
    t1.add_edge(1, 2)
    t1.add_edge(1, 3)

    for n in range(2, 5, 2):
        t2.add_node(test_nodes[n])
    t2.add_edge(1, 2)

    newT = RRT.merge_from_trees([t1, t2])
    logger.debug(f"nodes: {newT.get_nodes()}")
    logger.debug(f"edges: {newT.get_edges()}")


def test_get_route_info():
    tmp = RRT(test_origin, test_target)
    for n in range(3):
        tmp.add_node(test_nodes[n])
    tmp.add_edge(1, 2)
    tmp.add_edge(1, 3)
    logger.debug(f"nodes: {tmp.get_nodes()}")
    logger.debug(f"edges: {tmp.get_edges()}")
    route_info = tmp.get_route(test_nodes[0], test_nodes[2])
    logger.debug(f"path: {route_info.get_route()}")
    logger.debug(f"coordination: {route_info.get_route(route_type='coord')}")
    logger.debug(f"length: {route_info.get_length()}")
