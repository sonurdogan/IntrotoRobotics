from breezyslam.sensors import XVLidar as LaserModel
from breezyslam.algorithms import  RMHC_SLAM
from roboviz import MapVisualizer
from sys import  exit
from time import sleep
import numpy as np
import json

f = open('test.json')

data = json.load(f)
lidar_data=data['lidar']
f.close()

map_size_pixel = 500
map_size_meter = 8

seed = 999

mapbytes = bytearray(map_size_pixel * map_size_pixel)

slam = RMHC_SLAM(LaserModel(), map_size_pixel, map_size_meter, random_seed=seed) \

viz = MapVisualizer(map_size_pixel, map_size_meter)

pose = [0, 0, 0]

prevtime = 0


for i,j in lidar_data:
    # j stores timestamps

    slam.update(list(i))

    pose[0], pose[1], pose[2] = slam.getpos()
    
    slam.getmap(mapbytes)

    if not viz.display(pose[0]/1000, pose[1]/1000, pose[2]/1000, mapbytes):
        exit(0)
    
    currentime = j
    
    if (currentime-prevtime)<1:
        sleep(currentime - prevtime)
    else:
        sleep(1/5)

    prevtime = currentime
