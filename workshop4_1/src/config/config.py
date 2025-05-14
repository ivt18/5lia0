#!/usr/bin/env python2

import yaml

from datatypes import CarConfig

def read_config():
    with open("config.yml") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            return {}

def get_car_config():
    config = read_config()
    return CarConfig(config["wheel_radius"], config["wheelbase"])

def get_wheel_encoder_config():
    config = read_config()
    return config["wheel-encoder"]
