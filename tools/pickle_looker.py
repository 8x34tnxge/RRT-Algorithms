import argparse
import os
import pickle

import matplotlib.pyplot as plt


def get_opt() -> argparse.Namespace:
    parser = argparse.ArgumentParser("a tool to look 2D or 3D figure that was draw with matplotlib and pickled into .pickle file")
    parser.add_argument('-d', '--dir', dest='dir', default='./', help='the directory of the file')
    parser.add_argument('-n', '--name', dest='name', required=True, help="the name of the file")

    return parser.parse_args()

def show_figure(fig):
    dummy = plt.figure()
    new_manager = dummy.canvas.manager
    new_manager.canvas.figure = fig
    fig.set_canvas(new_manager.canvas)

if __name__ == '__main__':
    opt = get_opt()
    with open(os.path.join(opt.dir, opt.name), 'rb') as f:
        fig = pickle.load(f)
    show_figure(fig)
    # fig.show()
    plt.show()
