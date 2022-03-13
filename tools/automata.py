import os
from tqdm import trange

## ArgumentParser ##
import argparse

parser = argparse.ArgumentParser(
    description=\
    """
    this program is used to randomly generate 2d/3d maps and use them to run the current algorithms,
    and record the running result, for instance the runtime, relevant result data and save them into
    a data-store file.
    """,
)

parser.add_argument(
    '-d',
    '--dim',
    dest='dim',
    default='2d',
    required=True,
    choices=['2d', '3d'],
    help='the number of dimension'
)

parser.add_argument(
    '-n',
    '--num',
    dest='num',
    default=200,
    type=int,
    required=True,
    help='the number of test times'
)

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

parser.add_argument(
    "-b",
    "--boundary",
    dest="boundary",
    nargs="+",
    required=True,
    metavar=[2, 3, 4],
    help="the integer boundary started from 0 in x, y, z axis",
)

parser.add_argument(
    "--block-prob",
    dest='block_prob',
    type=float,
    default=0.2,
    help="the probability of block generation"
)

parser.add_argument(
    '--dir',
    dest='dir',
    type=str,
    default='./RRT/config/map',
    help='the directory of map files'
)

parser.add_argument(
    '-a', '--alg-dir',
    dest='alg_dir',
    type=str,
    default='./demo',
    help='the directory of alg demos'
)

parser.add_argument(
    '-t',
    '--type',
    dest='type',
    default='csv',
    help='the type of the data-store file'
)

parser.add_argument(
    '-o',
    '--output',
    dest='output',
    default='result.csv',
    help='the dir & name of the data-store file'
)

args = parser.parse_args()
if len(args.boundary) == 1 or len(args.boundary) > 3:
    parser.error("the boundary must be at least one value and no more than three values")

## Map Generation ##
map_create_iterator = trange(args.num, desc='Map Generating')
for n in map_create_iterator:
    os.system(' '.join([
        'pipenv', 'run', 'python',
        './tools/map_generator.py',
        '-t', str(args.dim),
        '--wall' if args.wall else '--no-wall',
        '-b', *map(str, args.boundary),
        '--block-prob', str(args.block_prob),
        '-o', os.path.join(args.dir, f'test_{n+1}.yaml')
        ]))

## Algorithm Running ##
alg_list = os.listdir(args.alg_dir)
alg_name_list = list(map(lambda x: x.rsplit('.')[0], alg_list))

for map_id in range(args.num):
    for alg_name, alg_file in zip(alg_name_list, alg_list):
        print(alg_name)
        if alg_name == 'basic_RRT':
            continue
        # [ ] change the const into args
        os.system(' '.join([
            'pipenv', 'run', 'python',
            os.path.join(args.alg_dir,alg_file),
            '-m', f'test_{map_id}',
            '-s', '3.',
            '-a', '1000'
        ]))

## Data Save ##

## LaTex Conversion ##

