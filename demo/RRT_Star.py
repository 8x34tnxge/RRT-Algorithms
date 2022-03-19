import argparse

from RRT.algorithm.RRT_star import RRT_Star
from RRT.config import map_loader
from RRT.core.mission_info import MissionInfo
from RRT.core.map_space import MapSpace
from RRT.core.sign import Status

from demo.util import save_result


## ArgumentParser ##
def get_opt():
    parser = argparse.ArgumentParser(
        description="""This program is used to run the basic-RRT algorithm using the given params and map.
        """
    )

    parser.add_argument(
        "-m", "--map", dest="map", required=True, help="the map name without postfix."
    )

    parser.add_argument(
        "-p",
        "--prob",
        dest="prob",
        default=0.2,
        type=float,
        help="the probability to explore an area of random direction",
    )

    parser.add_argument(
        "-s",
        "--step-size",
        dest="step_size",
        default=1,
        type=float,
        help="the size of each step. Default is 1",
    )

    parser.add_argument(
        "-a",
        "--attempt",
        dest="attempt",
        default=0,
        type=int,
        help="the attempt times. if less than or equal to 0, it will be positive infinity.",
    )

    parser.add_argument(
        "-o",
        "--output",
        dest="output",
        default="./output",
        help="the output directory to store the img data and route data.",
    )

    args = parser.parse_args()
    if args.prob > 1 or args.prob < 0:
        parser.error("the prob must be [0, 1]")

    return args


## Algorithm Running ##
def get_alg(map_name, prob, step_size, max_attempts, neighbor_num=5, *args, **kwargs):
    mission_info = MissionInfo(MapSpace(map_loader.get_map(map_name)))
    alg: RRT_Star = RRT_Star(
        None,
        mission_info,
        prob,
        step_size = step_size,
        max_attempts=max_attempts,
    )

    return alg


def main():
    args = get_opt()

    alg = get_alg(args.map, args.prob, args.step_size, args.attempt)
    status = alg.run()

    if status == Status.Success:
        route_info = alg.get_route()
        save_result(args.map, alg, route_info, args.output)


if __name__ == "__main__":
    main()
