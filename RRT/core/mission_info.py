from typing import Any

import numpy as np
from nptyping import NDArray
from RRT.core.map_space import MapSpace


class MissionInfo:
    def __init__(self, map_info: MapSpace):
        self.map_info: MapSpace = map_info

        self.origin = self.extract_origin_info()
        self.target = self.extract_target_info()

    def extract_origin_info(self) -> NDArray[Any]:
        """the instance method to extract the origin infomation/coordination from the given map info

        Returns
        -------
        NDArray[Any]
            the origin infomation/coordination
        """
        coord = np.nonzero(self.map_info.map == 1)
        origin = np.array(coord).reshape([-1])
        return origin

    def extract_target_info(self) -> NDArray[Any]:
        """the instance method to extract the target infomation/coordination from the given map info

        Returns
        -------
        NDArray[Any]
            the target information/coordination
        """
        coord = np.nonzero(self.map_info.map == 2)
        target = np.array(coord).reshape([-1])
        return target
