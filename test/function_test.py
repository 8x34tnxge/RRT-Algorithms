import numpy as np
from loguru import logger
from RRT.util.comb import combination_from_candidates


def test_comb():
    candidates = [[1, 2, 3], [1, 2], [2, 3, 4]]
    result = combination_from_candidates(candidates)
    answer = np.array(
        [
            [1, 1, 2],
            [1, 1, 3],
            [1, 1, 4],
            [1, 2, 2],
            [1, 2, 3],
            [1, 2, 4],
            [2, 1, 2],
            [2, 1, 3],
            [2, 1, 4],
            [2, 2, 2],
            [2, 2, 3],
            [2, 2, 4],
            [3, 1, 2],
            [3, 1, 3],
            [3, 1, 4],
            [3, 2, 2],
            [3, 2, 3],
            [3, 2, 4],
        ]
    )
    assert np.all(answer == np.array(result))
