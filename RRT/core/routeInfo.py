from typing import List, Union

import numpy as np


class RouteInfo:
    """the route infomation"""

    ######################################
    ############# properties #############
    ######################################
    _sequence: List[np.int32]
    _coords: List[List[np.float64]]
    _length: np.float64

    #########################################
    ############# class methods #############
    #########################################
    def __init__(
        self,
        sequence: List[np.int32],
        coords: List[List[np.float64]],
        length: np.float64,
    ):
        """the initial method for RouteInfo

        Args:
            sequence (List[np.int32]): the list of point/node ID in order.
            coords (List[List[np.float64]]): the multi-dim list of coordination of points/nodes respectively.
            length (np.float64): the length of the route
        """
        self._sequence = sequence
        self._coords = coords
        self._length = length

    def getRoute(
        self, type: str = "sequence"
    ) -> Union[List[np.int32], List[List[np.float64]]]:
        """the method to get the route infomation

        Args:
            type (str, optional): the type of the route infomation, only to "sequence" or "coord". Defaults to "sequence".

        Raises:
            ValueError: the Variable [type] is not "sequence" or "coord".

        Returns:
            Union[List[np.int32], List[List[np.float64]]]: the route result based on the given type
        """
        if type == "sequence":
            return self._sequence
        elif type == "coord":
            return self._coords
        else:
            raise ValueError(
                f"The variable type must be 'sequence' or 'coord'! its value is [{type}] now."
            )

    def getLength(self) -> np.float64:
        return self._length

    def isFeasible(self) -> bool:
        """the method to get the result of whether the current solution is feasible

        Returns:
            bool: the result of whether the current solution is feasible
        """
        if self._sequence is None or self._coords is None or self._length is None:
            return False
        else:
            return True
