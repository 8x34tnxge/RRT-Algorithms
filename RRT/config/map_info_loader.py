import os
from typing import Text
from yacs.config import CfgNode
from RRT.config.config_loader import ConfigLoader
from loguru import logger

class MapInfoLoader(ConfigLoader):
    def __init__(self, ):
        self.container = {}

    def load_map(self, map_file_name: Text):
        file_name = os.path.basename(map_file_name).rsplit(".", maxsplit=1)[0]
        assert file_name not in self.container.keys()

        config = CfgNode()
        config.MAP = None
        config.merge_from_file(map_file_name)

        self.container[file_name] = config.MAP

    def get_map(self, map_name):
        assert map_name in self.container.keys()
        return self.container[map_name]