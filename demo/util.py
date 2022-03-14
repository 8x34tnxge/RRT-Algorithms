import os
import pickle

import matplotlib
import pandas as pd


def save(file, obj):
    save_path = file.split('/')
    save_dir = save_path[:-1]
    save_file_name = save_path[-1]

    # make sure the dir exist
    for i in range(len(save_dir)):
        curr_dir = os.path.join(*save_dir[:i+1])
        if not os.path.exists(curr_dir):
            os.mkdir(curr_dir)

    if save_file_name.rsplit('.')[-1] == 'pickle':
        with open(file, 'wb') as f:
            f.write(pickle.dumps(obj))
    elif isinstance(obj, matplotlib.figure.Figure):
        obj.savefig(file)
    elif isinstance(obj, pd.DataFrame):
        obj.to_csv(file, sep=',')
