import time
from f110_gym.envs.base_classes import Integrator
import yaml
import gym
import numpy as np
from argparse import Namespace
from laser_models import ScanSimulator2D as Scan
from numba import njit
from pyglet.gl import GL_POINTS
import math

class leftwallPlanner:
    """
    Outside wall follower!!!
    Its for the paper trust
    """
    
    def __init__(self, conf, num_beams = 270):
        scanner = Scan(num_beams, np.pi*2*(3/4))
        scanner.set_map(conf.map_path + ".yaml", conf.map_ext)
        self.lidar = scanner
        self.num_beams = num_beams

    def plan(self, pose_x, pose_y, pose_theta):
        scanResultFull = self.lidar.scan(np.array([pose_x, pose_y, pose_theta]), None)[::-1]#[:self.num_beams//2]
        scanResult = scanResultFull[:135]

        # THIS IS ALL ASSUMING THE RIGHT SIDE OF THE CAR IS THE OUTSIDE CURVE


        #two options, could improve the old one or go with the 90 follower
        #they will both run into the same wobbling problem, so maybe ill fix current

        #using the 270 degree scanner

        #Figure out how to scale values to varying width of track
        driftLeft = False
        # print(pose_x, end=" ")
        defaultSpeed = 6

        if(scanResult[89]>2): #and (scanResult[134] < 5 or scanResult[0] > 1.5)):
            # if scanResult[134]<6:
            #     print("exit 1")
            #     return 4, -0.8
            print('exit 1')
            return defaultSpeed-3, 0.5
            
        
        if(scanResult[89]>1.3):
            driftRight = False
        
        if min(scanResult) < 0.4:
            print("exit 2")
            return defaultSpeed-2, -0.25
        
        if(scanResult[134]<2):
            print("wall incoming")
            return defaultSpeed-3, -0.8
        
        if(scanResult[134] < 5):
            driftLeft = True

            

        #find optangle
        mindex = np.argmin(scanResult) + 1
        # deviation = ((scanResult[0]-scanResult[89]) )
        deviation = (math.radians(45) - math.atan(scanResult[0]/scanResult[89])) 
        # print(math.atan(scanResult[0]/scanResult[89]))
        maxdev = 0.3
        scaleddeviation = max(min(deviation, maxdev), maxdev * -1)
        if scaleddeviation != deviation:
            deviation = 0
        speed = 4 + 7 // (abs(mindex-32.79) + 0.1)
        # if mindex - 45 > -5 or mindex-45 <5:
        #     print("opt found", deviation)
        # print(deviation)
        print(driftLeft)
        if driftLeft:
            deviation -= 0.1
        return defaultSpeed, deviation * 1.4



        #would have been really cool if it worked
        # maxTurn = 0.2
        # x = min(scanResult)
        # if x < 0.75:
        #     deviation = x-0.75
        # elif x >1.25:
        #     deviation = x-1.25
        # else:
        #     deviation = (x-1)*(1*(x-0.75))*(1*(x-1.25))*(20*(x-0.9))*(20*(x-1.1))
        # deviation = min(deviation, maxTurn) # bounds
        # deviation = max(deviation, maxTurn * -1) # can be condensed to one line
        # maxSpeed = 6
        # minSpeed = 3
        # speed = -20*((x-1)**2)+11.5 - 4
        # speed = min(speed, maxSpeed)
        # speed = max(speed, minSpeed)
        # print(-1*deviation, speed)
        # return speed, -deviation
        




        #This works, but is pretty bad rn
        #scanner = Scan(180, np.pi)
        leeway = 0.2
        maxTurn = 0.25
        if min(scanResult) > 1+leeway:
            turn = min((min(scanResult) - 1), maxTurn)
            #print(turn)
            return 2, -1*turn # negative number turns right
        elif min(scanResult) < 1-leeway:
            turn = min((1 - min(scanResult)), maxTurn)

            return 2, turn #positive number turns left
        else:
            return 6, 0



        