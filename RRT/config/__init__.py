import os

from RRT.config.config_update import config_update
from RRT.config.map_info_loader import MapInfoLoader
from RRT.config.universal_config_loader import UniversalConfigLoader

# universal config loader
config_dir = "./RRT/config/"
config_file_name = "base-config.yaml"
config_loader = UniversalConfigLoader()
config_update(config_dir, config_file_name, os.path.join(config_dir, "map"))
config_loader.appendConfig(os.path.join(config_dir, "base-config.yaml"))

# map info loader
map_loader = MapInfoLoader()
for map_file_name in config_loader.get_attr().SYSTEM.MAP:
    map_loader.load_map(os.path.join(config_dir, "map", map_file_name))

# extern function to get above singleton or their attributes
def get_config() -> UniversalConfigLoader:
    """return config loader singleton

    Returns
    -------
    UniversalConfigLoader
        config loader singleton
    """
    return config_loader


def get_map_config() -> MapInfoLoader:
    """return map loader singleton

    Returns
    -------
    MapInfoLoader
        config map singleton
    """
    return map_loader
