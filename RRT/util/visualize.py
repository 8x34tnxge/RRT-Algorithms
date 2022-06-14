import logging
from typing import Any

logging.getLogger("matplotlib").setLevel(logging.WARNING)
logging.getLogger("PIL").setLevel(logging.WARNING)

import matplotlib.figure
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch
from nptyping import NDArray
from RRT.config import get_config
from RRT.core.mission_info import MissionInfo
from RRT.core.route_info import RouteInfo

plt.set_loglevel("info")


def visualize(
    mission_info: MissionInfo, route_info: RouteInfo
) -> matplotlib.figure.Figure:
    """the method to visualize the result based on the mission info and route info, and save it to file

    Parameters
    ----------
    mission_info : MissionInfo
        the mission infomation
    route_info : RouteInfo
        the route information

    Raises
    ------
    ValueError
        the number of dimension must be 2 or 3!
    """
    # logger.debug(route_info.get_route('coord'))
    config = get_config()
    atlas = mission_info.map_info.map
    ndim = atlas.ndim
    if ndim == 2:
        fig = visualize_2d(atlas, route_info)
    elif ndim == 3:
        fig = visualize_3d(atlas, route_info)
    else:
        raise ValueError("the number of dimension must be 2 or 3!")

    return fig


def visualize_2d(
    atlas: NDArray[Any], route_info: RouteInfo
) -> matplotlib.figure.Figure:
    """the method to draw 2D figure

    Parameters
    ----------
    atlas : NDArray[Any]
        the map array
    route_info : RouteInfo
        the route info

    Returns
    -------
    matplotlib.figure.Figure
        the 2D figure
    """
    fig = plt.figure()
    ax = plt.gca()
    # add atlas info
    im = plt.imshow(
        (atlas == 3).astype(int),
        interpolation="none",
        vmin=0,
        vmax=1,
        cmap="Greys",
        aspect="equal",
    )

    # Route Section
    coordination = route_info.get_route()
    route = plt.plot(coordination[:, 1], coordination[:, 0], color="C1")

    # add origin info
    x, y = np.nonzero(atlas == 1)
    origin = plt.Circle((y, x), 0.2, color="r")
    ax.add_patch(origin)

    # add target info
    x, y = np.nonzero(atlas == 2)
    target = plt.Circle((y, x), 0.2, color="b")
    ax.add_patch(target)

    # Axis Section
    # empty the axis infomation
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    # Legend Section
    legend_handles = [
        Patch(facecolor="k", edgecolor="w"),
        origin,
        target,
        Patch(facecolor="C1", edgecolor="w"),
    ]
    fig.legend(handles=legend_handles, labels=["Wall", "Origin", "Target", "Route"])
    #plt.title(f"Length: {route_info.get_length()}")
    plt.title("")

    return fig


def visualize_3d(atlas: NDArray[Any], route_info: RouteInfo):
    """the method to draw 3D figure

    Parameters
    ----------
    atlas : NDArray[Any]
        the map array
    route_info : RouteInfo
        the route info

    Returns
    -------
    matplotlib.figure.Figure
        the 3D figure
    """
    fig = plt.figure()
    ax = plt.subplot(111, projection="3d")

    # Route Section
    coordination = route_info.get_route()
    (route,) = ax.plot(
        coordination[:, 0],
        coordination[:, 1],
        coordination[:, 2],
        color="C1",
        label="Route",
    )

    # # add origin info
    coord = np.nonzero(atlas == 1)
    origin = ax.scatter(coord[0], coord[1], coord[2], c="r", label="Origin")

    # # add target info
    coord = np.nonzero(atlas == 2)
    target = ax.scatter(coord[0], coord[1], coord[2], c="b", label="Target")

    # render the color of buildings
    wall_alpha = 1
    empty_alpha = 0

    colors = np.empty(list(atlas.shape) + [4])
    wall_x, wall_y, wall_z = np.nonzero(atlas == 3)
    colors[:] = [0, 0, 0, empty_alpha]
    for x, y, z in zip(wall_x, wall_y, wall_z):
        colors[x, y, z] = [1, 1, 1, wall_alpha]

    ax.voxels(atlas, facecolors=colors, label="Building")

    # add the legend of above things
    legend_handles = [
        Patch(facecolor="k", edgecolor="w"),
        route,
        origin,
        target,
    ]
    fig.legend(
        handles=legend_handles,
        labels=[
            "Building",
            "Route",
            "Origin",
            "Target",
        ],
    )

    # remove the axes info
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_zticklabels([])

    return fig
