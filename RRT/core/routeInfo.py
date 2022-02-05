from typing import List, Union
import numpy as np

class RouteInfo:
    _sequence: List[np.int32]
    _coords: List[List[np.float64]]
    _length: np.float64
    
    def __init__(self, sequence: List[np.int32], coords: List[List[np.float64]], length: np.float64):
        self._sequence = sequence
        self._coords = coords
        self._length = length

    def getRoute(self, type: str = "sequence") -> Union[List[np.int32], List[List[np.float64]]]:
        if type == "sequence":
            return self._sequence
        elif type == "coord":
            return self._coords
        else:
            raise ValueError(f"The variable type must be 'sequence' or 'coord'! its value is [{type}] now.")

    def getLength(self) -> np.float64:
        return self._length

    def isFeasible(self) -> bool:
        if self._sequence is None or self._coords is None or self._length is None:
            return False
        else:
            return True