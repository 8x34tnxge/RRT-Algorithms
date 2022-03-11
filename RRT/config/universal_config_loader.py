from typing import Text

from RRT.config.config_loader import ConfigLoader
from yacs.config import CfgNode


class UniversalConfigLoader(ConfigLoader):
    def __init__(
        self,
    ):
        self.config = CfgNode(new_allowed=True)

    def appendConfig(self, config_file_name: Text):
        self.config.merge_from_file(config_file_name)

    def get_attr(self):
        return self.config
