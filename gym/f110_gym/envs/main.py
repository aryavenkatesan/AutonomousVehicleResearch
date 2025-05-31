from argparse import Namespace
import time
import gym
import numpy as np
import yaml
from base_classes import Integrator
from custom_builds.simple_planner import planner # the policy/motion planner that you create
from custom_builds.follow_the_gap import followTheGap
from custom_builds.algoTesting import testing
from custom_builds.rightside_wall_planner import rightwallPlanner
from custom_builds.leftside_wall_planner import leftwallPlanner
# from custom_builds.lidar_plannerUpgrade import widthFinder
from laser_models import ScanSimulator2D as Scan
import random

# instantiating the environment
#racecar_env = gym.make('f110_gym:f110-v0')

# instantiating your policy
work = {'mass': 3.463388126201571, 'lf': 0.15597534362552312, 'tlad': 0.82461887897713965, 'vgain': 1.375, 'num_beams': 540}#0.90338203837889}

with open('custom_builds/config.yaml') as file:
    conf_dict = yaml.load(file, Loader=yaml.FullLoader)
conf = Namespace(**conf_dict)

oldplanner = planner(conf, (0.17145+0.15875)) #OLD PLANNER
# planner = testing(conf)
#planner = followTheGap(conf, work['num_beams'])
planner = leftwallPlanner(conf)
# planner = rightwallPlanner(conf)


scanner = Scan(270, np.pi*(3/2))
print(scanner.set_map(conf.map_path + ".yaml", conf.map_ext))

#print("Width", widthFinder(conf, conf.sx, conf.sy, conf.stheta))


def render_callback(env_renderer):
        # custom extra drawing function
        e = env_renderer

        # update camera to follow car
        x = e.cars[0].vertices[::2]
        y = e.cars[0].vertices[1::2]
        top, bottom, left, right = max(y), min(y), min(x), max(x)
        e.left = left - 800
        e.right = right + 800
        e.top = top + 800
        e.bottom = bottom - 800

        if obs['poses_x'][0] == -11.298575104323993: #for testing purposes
             res = (scanner.scan(np.array([obs['poses_x'][0], obs['poses_y'][0], obs['poses_theta'][0]]), None)[(270//2):])
             print((scanner.scan(np.array([obs['poses_x'][0], obs['poses_y'][0], obs['poses_theta'][0]]), None)[:135]))
             #print(res, obs['poses_x'][0], obs['poses_y'][0], obs['poses_theta'][0])



env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext, num_agents=1, timestep=0.01, integrator=Integrator.RK4)
env.add_render_callback(render_callback)

obs, step_reward, done, info = env.reset(np.array([[conf.sx, conf.sy, conf.stheta]]))
env.render()

# simulation loop
laptime = 0.0
start = time.time()

# loops when env not done
while not done:
    # get action based on the observation
    speed, steer = planner.plan(obs['poses_x'][0], obs['poses_y'][0], obs['poses_theta'][0])
    # speed, steer = planner.plan(obs['poses_x'][0], obs['poses_y'][0], obs['poses_theta'][0], scanner, 540)
    #speed, steer = planner.plan(obs['poses_x'][0], obs['poses_y'][0], obs['poses_theta'][0], work['tlad'], work['vgain'])

    # stepping through the environment
    obs, step_reward, done, info = env.step(np.array([[steer, speed]]))
    laptime += step_reward

    env.render(mode='human')


print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time()-start)
