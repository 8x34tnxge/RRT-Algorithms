import os
import pickle

import matplotlib
import matplotlib.pyplot as plt
import pandas as pd
from RRT.util.visualize import visualize


def save(file, obj):
    save_path = file.split("/")
    save_dir = save_path[:-1]
    save_file_name = save_path[-1]

    # make sure the dir exist
    for i in range(len(save_dir)):
        curr_dir = os.path.join(*save_dir[: i + 1])
        if not os.path.exists(curr_dir):
            os.mkdir(curr_dir)

    if save_file_name.rsplit(".")[-1] == "pickle":
        with open(file, "wb") as f:
            pickle.dump(obj, f)
    elif isinstance(obj, matplotlib.figure.Figure):
        obj.savefig(file)
        plt.close(obj)
    elif isinstance(obj, pd.DataFrame):
        obj.to_csv(file, sep=",")


def save_result(map_name, algorithm, route_info, output_dir='./output'):
    alg_name = algorithm.__class__.__name__
    save_name = f"{map_name}_{alg_name}"

    # save data
    fig = visualize(algorithm.mission_info, route_info)

    save(os.path.join(output_dir, f"img/{save_name}.png"), fig)
    save(os.path.join(output_dir, f"img_data/{save_name}.pickle"), fig)
    save(os.path.join(output_dir, f"route_info/{save_name}.pickle"), route_info)
