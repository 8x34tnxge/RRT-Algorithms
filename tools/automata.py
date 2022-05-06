## ArgumentParser ##
import argparse
import os
import time

import numpy as np
import pandas as pd
from tqdm import tqdm, trange
from RRT.core.sign import Status

parser = argparse.ArgumentParser(
    description="""
    this program is used to randomly generate 2d/3d maps and use them to run the current algorithms,
    and record the running result, for instance the runtime, relevant result data and save them into
    a data-store file.
    """,
)

parser.add_argument(
    "-d",
    "--dim",
    dest="dim",
    default="2d",
    required=True,
    choices=["2d", "3d"],
    help="the number of dimension",
)

parser.add_argument(
    "-n",
    "--num",
    dest="num",
    default=200,
    type=int,
    required=True,
    help="the number of test times",
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
    dest="block_prob",
    type=float,
    default=0.2,
    help="the probability of block generation",
)

parser.add_argument(
    "--dir",
    dest="dir",
    type=str,
    default="./RRT/config/map",
    help="the directory of map files",
)

parser.add_argument(
    "-a",
    "--alg-dir",
    dest="alg_dir",
    type=str,
    default="demo",
    help="the directory of alg demos",
)

parser.add_argument(
    "-t", "--type", dest="type", default="csv", help="the type of the data-store file"
)

parser.add_argument(
    "-o",
    "--output",
    dest="output",
    default="result.csv",
    help="the dir & name of the data-store file",
)

parser.add_argument(
    '--output-dir',
    dest="output_dir",
    default='./output',
    help="the root dir of output files"
)

args = parser.parse_args()
if len(args.boundary) == 1 or len(args.boundary) > 3:
    parser.error(
        "the boundary must be at least one value and no more than three values"
    )

## Map Generation ##
map_create_iterator = trange(args.num, desc="Map Generating")
for n in map_create_iterator:
    os.system(
        " ".join(
            [
                "pipenv",
                "run",
                "python",
                "./tools/map_generator.py",
                "-t",
                str(args.dim),
                "--wall" if args.wall else "--no-wall",
                "-b",
                *map(str, args.boundary),
                "--block-prob",
                str(args.block_prob),
                "-o",
                os.path.join(args.dir, f"test_{n+1}.yaml"),
            ]
        )
    )

## Algorithm Running ##
import importlib

alg_list = list(filter(lambda x: x not in ['__pycache__', 'util.py'], os.listdir(args.alg_dir)))
alg_name_list = list(map(lambda x: x.rsplit(".")[0], alg_list))
print("Import 'demo' as Module...")
alg_module_list = {
    alg_name: importlib.import_module(".".join([args.alg_dir, alg_name]))
    for alg_name in alg_name_list
}
print("Import Finished!")

# prepare for pandas DataFrame
algs = []
maps = []
costs = []
runtimes = []

alg_run_iterator = []
for map_id in range(args.num):
    map_name = f'test_{map_id+1}'
    for alg_name in alg_name_list:
        alg_run_iterator.append(tuple([map_name, alg_name]))
alg_run_iterator = tqdm(alg_run_iterator, desc='map: empty alg: empty')

for map_name, alg_name in alg_run_iterator:
    # update process bar
    alg_run_iterator.set_description(f"map: {map_name} alg: {alg_name}")

    # [ ] change the const into args
    maps.append(map_name)
    algs.append(alg_name)
    # record time unit: s
    alg_module = alg_module_list[alg_name]

    alg = alg_module.get_alg(
        map_name=map_name,
        prob=0.3,
        step_size=3,
        max_attempts=np.inf
    )
    start_time = time.time()

    status = alg.run()

    duration = time.time() - start_time

    # here 0 means the program is working with out error
    if status == Status.Success:
        runtimes.append(duration)
        route_info = alg.get_route()
        costs.append(route_info.get_length())
        alg_module.save_result(map_name, alg, route_info, args.output_dir)
    else:
        # runtimes.append(np.nan)
        runtimes.append(duration)
        costs.append(np.nan)

## Data Save ##
run_data = pd.DataFrame(
    {"Algorithm": algs, "Map": maps, "Cost": costs, "Runtime": runtimes}
)
run_data.to_csv("./output/data.csv", sep=",", index=False)

## LaTex Conversion ##
