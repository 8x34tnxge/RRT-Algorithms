from typing import Any

import numpy as np
from nptyping import NDArray
from RRT.core.mapinfo import MapInfo


class MissionInfo:
    def __init__(self, map_info: MapInfo):
        self.map_info = map_info

        self.origin = self.extract_origin_info()
        self.target = self.extract_target_info()

    def extract_origin_info(self) -> NDArray[Any]:
        """the instance method to extract the origin infomation/coordination from the given map info

        Returns
        -------
        NDArray[Any]
            the origin infomation/coordination
        """
        x, y = np.nonzero(self.map_info.map == 1)
        # each map should have only one origin
        assert x.shape[0] == 1 and y.shape[0] == 1

        origin = np.array([x, y]).reshape([-1])
        return origin

    def extract_target_info(self) -> NDArray[Any]:
        """the instance method to extract the target infomation/coordination from the given map info

        Returns
        -------
        NDArray[Any]
            the target information/coordination
        """
        x, y = np.nonzero(self.map_info.map == 2)
        # each map should have only one target
        assert x.shape[0] == 1 and y.shape[0] == 1

        target = np.array([x, y]).reshape([-1])
        return target
