## ArgumentParser ##
import argparse
import os
import pickle
import time

import numpy as np
import pandas as pd
from tqdm import trange

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

alg_run_iterator = trange(args.num * len(alg_list))
runtime_record = {}
for map_id in range(args.num):
    for alg_name, alg_file in zip(alg_name_list, alg_list):
        alg_run_iterator.desc = f'map: test_{map_id+1}\talg: {alg_name}'
        alg_run_iterator.update()
        if alg_name == 'basicRRT' or alg_name == 'util':
            continue
        # [ ] change the const into args
        # record time unit: s
        start_time = time.time()
        stat = os.system(' '.join([
            'pipenv', 'run', 'python',
            os.path.join(args.alg_dir,alg_file),
            '-m', f'test_{map_id+1}',
            '-s', '3.',
            '-a', '1000'
        ]))

        # here 0 means the program is working with out error
        if stat == 0:
            runtime_record[f'test_{map_id+1}_{alg_name}'] = time.time() - start_time
        else:
            runtime_record[f'test_{map_id+1}_{alg_name}'] = np.inf

## Data Save ##
route_info_list = os.listdir('./output/route_info')
route_info_name_list = list(map(lambda x: x.rsplit('.')[0], route_info_list))

algs = []
maps = []
lengths = []
runtimes = []

for map_id in range(args.num):
    for alg_name in alg_name_list:
        if alg_name in ['basicRRT', 'util']:
            continue
        algs.append(alg_name)
        maps.append(f'test_{map_id+1}')
        try:
            with open(os.path.join('./output/route_info', f'test_{map_id+1}_{alg_name}'+'.pickle'), 'rb') as f:
                route_info = pickle.load(f)
                lengths.append(route_info.get_length())
        except FileNotFoundError:
            lengths.append(np.inf)
        runtimes.append(runtime_record[f'test_{map_id+1}_{alg_name}'])

run_data = pd.DataFrame({
    "Algorithm": algs,
    "Map": maps,
    "Cost": lengths,
    "Runtime": runtimes
})
run_data.to_csv('./output/data.csv', sep=',', index=False)

## LaTex Conversion ##

