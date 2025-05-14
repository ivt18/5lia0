import yaml

from datatypes import CarConfig

def read_config() -> dict:
    with open("config.yml") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            return {}

def get_car_config() -> CarConfig:
    config: dict = read_config()
    return CarConfig(config["wheel_radius"], config["wheelbase"])