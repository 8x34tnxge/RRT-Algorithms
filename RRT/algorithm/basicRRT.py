from __future__ import annotations
from RRT.core import RRT_Template
from typing import Any, List
import numpy as np
import networkx as nx

class BasicRRT(RRT_Template):
    def __init__(self, origin: np.ndarray, target: np.ndarray) -> None:
        super().__init__(origin, target)

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        return super().__call__(*args, **kwds)

    def mergeFromTrees(cls, trees: List[BasicRRT]) -> BasicRRT:
        pass
