import time
from f110_gym.envs.base_classes import Integrator
import yaml
import gym
import numpy as np
from argparse import Namespace
from laser_models import ScanSimulator2D as Scan
from numba import njit
from pyglet.gl import GL_POINTS

class testing:
    def __init__(self, conf, num_beams=540):
        scanner = Scan(num_beams, np.pi)
        scanner.set_map(conf.map_path + ".yaml", conf.map_ext)
        self.lidar = scanner
        self.num_beams = num_beams

    def plan(self, pose_x, pose_y, pose_theta):




        
        return -1, -1