import argparse

import numpy as np
from RRT.algorithm.RRT_Connect import RRT_Connect
from RRT.config import map_loader
from RRT.core.info import MapInfo, MissionInfo
from RRT.util.visualize import visualize

from util import save

## ArgumentParser ##
parser = argparse.ArgumentParser(
    description=\
    """This program is used to run the basic-RRT algorithm using the given params and map.
    """
)

parser.add_argument(
    '-m', '--map',
    dest='map',
    required=True,
    help="the map name without postfix."
)

parser.add_argument(
    '-p', '--prob',
    dest='prob',
    default=0.2,
    type=float,
    help="the probability to explore an area of random direction"
)

parser.add_argument(
    '-s', '--step-size',
    dest='step_size',
    default=1,
    type=float,
    help="the size of each step. Default is 1"
)

parser.add_argument(
    '-a', '--attempt',
    dest='attempt',
    default=0,
    type=int,
    help='the attempt times. if less than or equal to 0, it will be positive infinity.'
)

parser.add_argument(
    '-o', '--output',
    dest='output_name',
    default='none',
    help='the output file name without postfix.'
)

args = parser.parse_args()
if args.prob > 1 or args.prob < 0:
    parser.error("the prob must be [0, 1]")


## Algorithm Running ##
mission_info = MissionInfo(
    MapInfo(map_loader.get_map(args.map))
)
alg: RRT_Connect = RRT_Connect(None, mission_info, args.prob, args.step_size, args.attempt if args.attempt > 0 else np.inf)
res = alg.run()

route_info = alg.get_route()

if args.output_name == 'none':
    alg_name = alg.__module__.split('.')[-1]
    save_name = f'{args.map}_{alg_name}'
else:
    save_name = args.output_name


fig = visualize(mission_info, route_info)

save(f'./output/img/{save_name}.png', fig)
save(f'./output/img_data/{save_name}.pickle', fig)
save(f'./output/route_info/{save_name}.pickle', route_info)
