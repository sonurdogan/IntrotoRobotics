from breezyslam.algorithms import  RMHC_SLAM
from mines import MinesLaser, Rover, load_data
from roboviz import MapVisualizer
from sys import  exit
from time import sleep

map_size_pixel = 1200
map_size_meter = 48

dataset = 'exp2'
use_odometry = True #note
seed = 999

mapbytes = bytearray(map_size_pixel * map_size_pixel)

timestamps, lidars , odometries= load_data('.', dataset)

slam = RMHC_SLAM(MinesLaser(), map_size_pixel, map_size_meter, random_seed=seed) \

robot = Rover()

viz = MapVisualizer(map_size_pixel, map_size_meter, dataset)

pose = [0, 0, 0]

prevtime = 0

for scan in range(len(lidars)):

    vel = robot.computePoseChange(odometries[scan])

    slam.update(lidars[scan],vel)
    
    #X,Y,Angle
    pose[0], pose[1], pose[2] = slam.getpos()

    slam.getmap(mapbytes)

    if not viz.display(pose[0] / 1000., pose[1] / 1000., pose[2], mapbytes):
        exit(0)

    currentime = timestamps[scan] / 1.e9
    if prevtime > 0:
        sleep(currentime - prevtime)
    prevtime = currentime 
