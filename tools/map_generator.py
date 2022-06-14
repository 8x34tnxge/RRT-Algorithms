import yaml
import numpy as np
import argparse
from typing import List, Any
from nptyping import NDArray
from RRT.core.sign import MapType


def get_opt():
    parser = argparse.ArgumentParser(
        "Generate random 2D or 3D map to standard yaml file"
    )

    # the dimension setting
    parser.add_argument(
        "-t",
        "--type",
        dest="type",
        default="2d",
        required=True,
        help="the number of dimension",
        choices=["2d", "3d"],
    )

    # if there is wall around the map
    parser.add_argument(
        "--wall",
        dest="wall",
        action="store_true",
        help="the map will be around the wall",
    )
    parser.add_argument(
        "--no-wall",
        dest="wall",
        action="store_false",
        help="the map will not be around the wall",
    )
    parser.set_defaults(wall=True)

    # x, y, z axis setting
    parser.add_argument(
        "-b",
        "--boundary",
        dest="boundary",
        nargs="+",
        required=True,
        metavar=[2, 3, 4],
        help="the integer boundary started from 0 in x, y, z axis",
    )

    # block/wall setting
    parser.add_argument(
        "--block-prob",
        dest='block_prob',
        type=float,
        default=0.2,
        help="the probability of block generation"
    )

    # output setting
    parser.add_argument(
        "-o",
        "--output",
        dest="output",
        nargs="?",
        default="map.yaml",
        help="the output position of yaml file",
    )

    args = parser.parse_args()
    if len(args.boundary) == 1 or len(args.boundary) > 3:
        parser.error("the boundary must be at least one value and no more than three values")

    return args


def generate_2d_map(opt: argparse.Namespace) -> NDArray[(Any, Any)]:
    boundary = np.array(list(map(int, opt.boundary)))[:2]
    if opt.wall is True:
        boundary -= 2

    # basic map generation
    ret = np.random.choice([MapType.EMPTY, MapType.WALL], size=boundary, p=[1 - opt.block_prob, opt.block_prob])
    if opt.wall is True:
        ret = np.pad(ret, 1, constant_values=MapType.WALL)

    # block/obstacle/wall generation
    # rndProb = np.random.random(size=(np.count_nonzero(ret == EMPTY)))
    # empty_x, empty_y = np.nonzero(ret == EMPTY)
    # for id in range(rndProb.shape[0]):
    #     if rndProb[id] < opt.block_prob:
    #         ret[empty_x[id], empty_y[id]] = WALL

    # origin & target generation
    empty_num = np.count_nonzero(ret == MapType.EMPTY)
    empty_x, empty_y = np.nonzero(ret == MapType.EMPTY)

    origin_id= np.random.choice(range(empty_num), 1)
    available_choice = []
    for available_id, (x, y) in enumerate(zip(empty_x, empty_y)):
        dist = np.linalg.norm(np.array([empty_x[origin_id] - x, empty_y[origin_id] - y]))
        if dist >= boundary.min() / 2:
            available_choice.append(available_id)
    target_id = np.random.choice(available_choice, 1)

    ret[empty_x[origin_id], empty_y[origin_id]] = MapType.ORIGIN
    ret[empty_x[target_id], empty_y[target_id]] = MapType.TARGET

    return ret


# [ ] only consider buildings now
def generate_3d_map(opt):
    boundary = np.array(list(map(int, opt.boundary)))[:3]

    # basic map generation
    ret = np.ones(boundary) * MapType.EMPTY

    # block/obstacle/wall generation
    empty_num = np.count_nonzero(ret[:, :, 0] == MapType.EMPTY)
    rndProb = np.random.random(size=(empty_num))
    rndHeight = np.random.randint(boundary[2]+1, size=(empty_num))
    empty_x, empty_y = np.nonzero(ret[:, :, 0] == MapType.EMPTY)
    for id in range(rndProb.shape[0]):
        if rndProb[id] < opt.block_prob:
            ret[empty_x[id], empty_y[id], :rndHeight[id]+1] = MapType.WALL

    # origin & target generation
    empty_num = np.count_nonzero(ret[:, :, 0] == MapType.EMPTY)
    empty_x, empty_y = np.nonzero(ret[:, :, 0] == MapType.EMPTY)
    origin_id, target_id = np.random.choice(range(empty_num), 2, replace=True)

    ret[empty_x[origin_id], empty_y[origin_id], 0] = MapType.ORIGIN
    ret[empty_x[target_id], empty_y[target_id], np.random.randint(boundary[2])] = MapType.TARGET

    return ret

def save_yaml(opt: argparse.Namespace, atlas: NDArray[Any]):
    with open(opt.output, 'w',encoding='utf-8') as f:
        yaml.dump({
            'MAP': atlas.tolist(),
        }, f, default_flow_style=None)


if __name__ == "__main__":
    opt = get_opt()

    if opt.type == '2d' or opt.type == '2D':
        ret = generate_2d_map(opt)
    elif opt.type == '3d' or opt.type == '3D':
        ret = generate_3d_map(opt)
    # [ ] deal with the rest situation

    save_yaml(opt, ret)
