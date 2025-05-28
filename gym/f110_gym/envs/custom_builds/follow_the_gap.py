import time
from f110_gym.envs.base_classes import Integrator
import yaml
import gym
import numpy as np
from argparse import Namespace
from laser_models import ScanSimulator2D as Scan
from numba import njit
from pyglet.gl import GL_POINTS

@njit(fastmath=False, cache=True)
def widthFinder(scanResult): 
    """ 
    Takes a 360 degree scan of the current position 
    Returns the smallest diameter- approximating the width of the track at that point
    
        Args: 
            scanResult (numpy.ndarray (360, )): scan of current x and y coordinates with 360 deg fov and one scan for every degree
        
        Returns: 
            minDiameter (float): said minimum diameter of the scan
    """ 
    res = scanResult 
    minDiameter = 9999.0 
    for index in range(180): 
        diameter = res[index] + res[index + 180] 
        if diameter < minDiameter: 
            minDiameter = diameter

    return minDiameter

@njit(fastmath=False, cache=True)
def chainCheck(scanResult, num_beams, limit, threshold = 30, fov=0.8):
    """
    Checks whether there are a consecutive number of scan values that exceed the threshold
    If there is then find the center of that consecutive chain and return a deviation along with True
    This method will return True when the ego is going along a straight and helps prevent wobbling

        Args:
            scanResult (numpy.ndarray (n, )): data array of the laserscan, n=num_beams
            num_beams (int): size of the scan
            limit (int): size in degrees of how long the longest chain has to exceed for True to return
            threshold (float): how high does a scan element have to be to be counted in the chain
            fov (float): value between 0-1 that denotes what part of the scan is being considered
                         0 is entire scan, 0.5 is the center 90 degrees
                         This helps with smoothing of the turns
        
        Returns: 
            isChain (Boolean): is there a chain with length over the limit
            deviation (float): angle the car has to change to go to the center of the chain
                               -1 if N/A
    """
    current30chain = 0 
    longest30chain = 0 
    longest30chainIndex = 0 
    limit = round(num_beams/180) * limit
    for index in range(round(num_beams*fov/2), round(num_beams*(1-fov/2)), 1): 
        if scanResult[index] >= threshold: 
            if current30chain == 0: 
                longest30chainIndex = index 
            if current30chain > longest30chain: 
                longest30chain = current30chain 
            current30chain += 1 
        else: 
            current30chain = 0
    
    if longest30chain > limit: 
        optAngleIndex = longest30chainIndex + longest30chain // 2 
        deviation = (optAngleIndex - num_beams/2)*(np.pi/180) * 180/num_beams 
        return True, deviation
    
    return False, -1


@njit(fastmath=False, cache=True)
def createBubbles(scanResult, num_beams, margin = 15, sensitivity = 2.5):
    """
    Creates protective "bubbles" around corners
    Does this by replacing scan values around a corner with 0

        Args:
            scanResult (numpy.ndarray (n, )): data array of the laserscan, n=num_beams
            num_beams (int): size of the scan
            margin (int): amount of degrees should turn to 0 around a corner
            sensitivity (float): minimum difference between two adjacent beams for a shutdown to occur

        Returns:
            scanResult (numpy.ndarray (n, )): data array of the laserscan, n=num_beams
                                              this data array has replaced values with 0s around corners
    """
    # margin = round(num_beams/180) * 15 # how many degrees should be shut down
    # sensitivity = 2.5 # what is the min difference between two adjacent beams for a shutdown to occur
    margin = round(num_beams/180) * margin
    for index in range(margin, num_beams-margin, 1): #find the edges and make a protective bubble
        if abs(scanResult[index-1] - scanResult[index]) > sensitivity and scanResult[index-1] != 0 and scanResult[index] != 0:
            if min(scanResult[index-1], scanResult[index]) > 10:
                margin //= 2
            for m in range (margin):
                scanResult[index + m], scanResult[index - m] = 0, 0
            scanResult[index] = 0

    return scanResult

@njit(fastmath=False, cache=True)
def findOptimalAngle(scanResult, num_beams, fov):
    """
    Returns the index of the highest value of the selected selection of the scan

        Args:
            scanResult (numpy.ndarray (n, )): data array of the laserscan, n=num_beams
            num_beams (int): size of the scan
            fov (float): value between 0-1 that denotes what part of the scan is being considered
                         0 is entire scan, 0.5 is the center 90 degrees
                         This helps with the car not going back where it came from when turning

        Returns:
            optAngleIndex (int): index of the highest value of the selected selection of the scan
    """
    optimalAngle = 0
    optAngleIndex = 0
    for index in range(round(num_beams*fov/2), round(num_beams*(1-fov/2)), 1): #make 180 a variable called numBeams for standardization
        if optimalAngle <= scanResult[index]:
            optAngleIndex = index
            optimalAngle = scanResult[index]

    return optAngleIndex

@njit(fastmath=False, cache=True)
def getActuation(scanResult, num_beams, optAngleIndex, maxTurn, fov):
    """ 
    Finds the proper speed and turning angle given the arguments 
    This can be optimized more: 
        Comment 1: speed is more or less arbitraily generated 
                   there are many situations in which it could be increased drastically 
        Comment 2: meant for smoothing, but sometimes it gives an incorrect angle 

        Args: 
            scanResult (numpy.ndarray (n, )): data array of the laserscan, n=num_beams 
            num_beams (int): size of the scan 
            optAngleIndex (int): index of the highest value of the selected selection of the scan 
            maxTurn (float): caps the absolute value of the (double) devation to smooth the turns out

        Returns: 
        speed (int): the ideal velocity of the car given the situation 
        deviation (float): angle the car has to change to go to furthest legal point 
    """
    deviation = (optAngleIndex - num_beams/2)*(np.pi/180) * 180/num_beams
    speed = 11 - np.abs(num_beams/2 - optAngleIndex) #1
    if speed < 4: 
        speed = 4
    if fov == 0.5: #2
        if deviation > maxTurn: 
            deviation = maxTurn
        if deviation < -1 * maxTurn:
            deviation = -1 * maxTurn
    return speed, deviation/2


class followTheGap:
    """
    Gap follower algorithm
    """
    def __init__(self, conf, num_beams = 540):
        self.conf = conf
        scanner = Scan(360, np.pi*2)
        scanner.set_map(conf.map_path + ".yaml", conf.map_ext)
        self.proximityScanner = scanner

        scanner = Scan(num_beams, np.pi)
        scanner.set_map(conf.map_path + ".yaml", conf.map_ext)
        self.lidar = scanner
        self.num_beams = num_beams

    
    def plan(self, pose_x, pose_y, pose_theta):
        """
        Plans a path towards the gap using lidar
        """
        #get scan data
        scanResult = self.lidar.scan(np.array([pose_x, pose_y, pose_theta]), None)
        fov = 0.5 if np.min(scanResult) < 1.5 else 0.0  

        #Check if good for max speed
        isChain, deviation = chainCheck(scanResult, self.num_beams, 3, 30, 0.8)
        if isChain:
            return 11, deviation
        
        #create protective bubbles around edges
        scanResult = createBubbles(scanResult, self.num_beams, 15, 2.5)

        #find the optimal angle
        optAngleIndex = findOptimalAngle(scanResult, self.num_beams, fov)

        #find and return changes to velocity and angle change
        return getActuation(scanResult, self.num_beams, optAngleIndex, 0.5, fov)