from RRT.config import get_config, get_map_config, get_test_map
from loguru import logger

def test_get_config():
    config = get_config()
    # logger.debug(config.config)

def test_map_config():
    map_config = get_map_config()
    # logger.debug(map_config.container)

def test_obtain_test_map():
    config = get_config()
    logger.debug(config.get_attr().TEST.MAP)
    map_info = get_test_map()
    # logger.debug(map_info)