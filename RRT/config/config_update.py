import os


def config_update(config_dir: str, config_file_name: str, map_dir: str):
    map_list = os.listdir(map_dir)
    with open(os.path.join(config_dir, config_file_name), "w") as config_file:
        config_file.write("SYSTEM:\n")
        config_file.write("  MAP: [\n")
        for map_file_name in map_list:
            config_file.write(f'    "{map_file_name}",\n')
        config_file.write("  ]\n")
        config_file.write('  SAVE_DIR: "./output"\n')
