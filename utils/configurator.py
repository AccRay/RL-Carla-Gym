import configparser


config = configparser.ConfigParser()
config.read('../config.ini')
# public
CARLA_DIRECTORY_PATH = config['public']['carla_directory_path']