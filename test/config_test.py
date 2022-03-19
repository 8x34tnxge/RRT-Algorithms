from RRT.config import get_config, get_map_config
from loguru import logger


def test_get_config():
    config = get_config()
    # logger.debug(config.config)


def test_map_config():
    map_config = get_map_config()
    # logger.debug(map_config.container)
