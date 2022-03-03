from RRT.core import RRT
from RRT.algorithm.RRT_template import RRT_Template
from RRT.core.info import RouteInfo


class RRT_Star(RRT_Template):
    def run(self) -> bool:
        """the method to run the basic RRT algorithm

        Returns
        -------
        bool
            whether basic RRT algorithm reach the target from origin
        """
        pass

    def get_route(self) -> RouteInfo:
        """the instance method to get route info

        Returns
        -------
        RouteInfo
            the route information containing the route from origin to target
        """
        pass

