# -*- coding: utf-8 -*-
import copy
import json
import logging
import math
import random
import sys
from tqdm import tqdm
from logging.config import dictConfig
from time import sleep
from typing import List
import yaml

import matplotlib.pyplot as plt
import numpy as np
import yaml
from triangulation.triangulator import Triangulator
from visualisation.visualiser import Visualiser
from visualisation.track import Track

from pathlib import Path

logging_config = dict(
    version=1,
    formatters={
        "with_time": {"format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s"},
        "without_time": {"format": "%(name)s - %(levelname)s - %(message)s"},
    },
    handlers={
        "debug_handler": {"class": "logging.StreamHandler", "formatter": "without_time", "level": logging.DEBUG},
        "warning_handler": {
            "class": "logging.FileHandler",
            "filename": "pathfinding.log",
            "formatter": "with_time",
            "level": logging.WARNING,
        },
    },
    root={
        "handlers": ["debug_handler", "warning_handler"],
        "level": logging.DEBUG,
    },
)

dictConfig(logging_config)


def main(params, algorithm):
    """
    Loads the track and runs the mapping and visualisation
    """

    # Read in the configuration file

    cwd = config_file = Path(__file__).absolute().parent
    track_path = cwd / params['track']

    track = Track(track_path, params['expand_dist'], params['plan_dist'])

    plt.ion()
    figure = plt.figure()
    viz = Visualiser(figure)

    """
    for i in tqdm(range(100000)):
        rel_cones = track.cones_car_space

        front_cones = track.get_front_cones_within_distance(rel_cones, params['plan_dist'])
        path, edge_centers, root = algorithm.get_path(front_cones)

        track.update_car_pos(path[0])
    input()
    """

    while True:
        rel_cones = track.cones_car_space

        front_cones = track.get_front_cones_within_distance(rel_cones, params['plan_dist'])
        #front_cones = track.add_cone_noise(front_cones, params['colour_prob'], params['pos_prob'], params['pos_radius'])
        # track.add_cone_noise(front_cones, params['colour_prob'], params['pos_prob'], params['pos_radius'])

        path, edge_centers = algorithm.get_path(front_cones, None)

        track.update_car_pos(path[0])

        node_list = [path[0]]
        visited_nodes = []

        edges = []

        while len(node_list) > 0:
            cur_node = node_list.pop(0)
            children = cur_node.children
            node_list.extend(children)

            for child in children:
                edges.append([[cur_node.x, cur_node.y], [child.x, child.y]])

        selected_path = []

        for cur_node, next_node in zip(path[:-1], path[1:]):
            selected_path.append([[cur_node.x, cur_node.y], [next_node.x, next_node.y]])

        # print(path)
        # print(selected_path)
      
        viz.visualise(rel_cones, front_cones, edge_centers, path[0], path)
        # input()
        #sleep(0.1)


if __name__ == "__main__":
    cwd = config_file = Path(__file__).absolute().parent
    config_file = cwd / 'config.yml'

    with open(config_file, 'r') as f:
        params = yaml.load(f)

    algorithm = Triangulator(
                params["triangulation_min_var"],
                params["triangulation_var_threshold"],
                params["max_iter"],
                params["max_angle_change"],
                params["max_path_distance"],
                params["safety_dist"],
                )
    main(params, algorithm)