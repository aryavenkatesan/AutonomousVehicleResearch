import time
from f110_gym.envs.base_classes import Integrator
import yaml
import gym
import numpy as np
from argparse import Namespace
from laser_models import ScanSimulator2D as Scan
from numba import njit
from pyglet.gl import GL_POINTS

class voronoiPlanner:
    def __init__(self, conf, num_beams=270):
        scanner = Scan(num_beams, np.pi*1.5)
        scanner.set_map(conf.map_path + ".yaml", conf.map_ext)
        self.lidar = scanner
        self.num_beams = num_beams

    def plan(self, pose_x, pose_y, pose_theta):
        scanResult = self.lidar.scan(np.array([pose_x, pose_y, pose_theta]), None)

        # try 90 degree feelers in the front so it sees there and can react earlier


        smallestValue = 999 
        smallestIndex = 0 #Angle the car is vs the optimal angle
        delta = 0 #How much bigger the left distance is than right distance
        for i in range(45, 135): # value is hardcoded
            value = scanResult[i] + scanResult[i+90]
            if value < smallestValue:
                smallestValue = value
                smallestIndex =  i
                delta = scanResult[i] - scanResult[i+90]
        
        deviation = (smallestIndex - 90) * 0.03
        if delta < -0.2:
            if delta > 0:
                deviation = 0
            deviation += 0.4
        elif delta > 0.2:
            if delta < 0:
                deviation = 0
            deviation -= 0.4
        
        baseSpeed = 5
        speed = baseSpeed
        if abs(smallestIndex-90) < 4:
            speed = 9

        
        return speed, deviation/3