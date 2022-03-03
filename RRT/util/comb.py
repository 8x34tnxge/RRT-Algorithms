from queue import SimpleQueue
from typing import Any, Callable, List


def combination_from_candidates(
    candidates: List[Any], converter: Callable = lambda x: x
) -> List[Any]:
    """the function to generate combinations of elements while each candidates only provide one element eachtime

    Parameters
    ----------
    candidates : List[Any]
        the list of each candidate
    converter : Callable, optional
        the converter to transfer the combination to ideal type, by default lambda x: x

    Returns
    -------
    List[Any]
        the combination result
    """
    # use the leaf node of tree structure using queue structure to implement
    leaf_num = 1
    combinations = SimpleQueue()

    for candidate in candidates:
        # extract each leaf node
        for _ in range(leaf_num):
            tmpComb = combinations.get() if combinations.empty() is not True else []
            # copy each leaf node for [element number] time and append the element in the end
            for elem in candidate:
                tmp = list()
                tmp.extend(tmpComb)
                tmp.append(elem)
                combinations.put(tmp)
        # recalculate the the number of leaf nodes
        leaf_num *= len(candidate)

    # use type converter to convert combination to ideal type
    res = []
    while combinations.empty() is not True:
        res.append(converter(combinations.get()))
    return res
