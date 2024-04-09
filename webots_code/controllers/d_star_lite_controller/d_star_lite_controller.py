# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, InertialUnit, GPS, DistanceSensor, Camera
import math 
from math import sin, cos, pi 
import heapq
import math
from scipy.optimize import minimize_scalar
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
import numpy as np

import sys 
sys.path.append('../../../')

from baselines.simplified_d_star_lite.grid import * 
from baselines.simplified_d_star_lite.d_star_lite import * 
from revised_version.utility_func import *

OBSTACLE = 255
UNOCCUPIED = 0

# include relevant import statements regarding path generation here.. 
GRID_SIZE = 0.2
ENV_LENGTH = 1
x_dim =  ENV_LENGTH // GRID_SIZE
y_dim = ENV_LENGTH // GRID_SIZE

# start and goal pos from actual env
start = (0, 0)
goal = (0.5, 0.5)
view_range = 5

# need way to convert cur pos to grid space pos 
startx, starty = real_to_grid_pos(real_pos=start, env_size=(ENV_LENGTH,ENV_LENGTH), upper_left_corner=(-0.5, 0.5), grid_size = GRID_SIZE)
goalx, goaly = real_to_grid_pos(real_pos=goal, env_size=(ENV_LENGTH,ENV_LENGTH), upper_left_corner=(-0.5, 0.5), grid_size = GRID_SIZE)

map  =  OccupancyGridMap(x_dim=x_dim,
                            y_dim=y_dim,
                            exploration_setting='4N')

new_position = start
last_position = start

# D* Lite (optimized)
print(f'new map for d* lite looks like: {map.occupancy_grid_map}')
dstar = DStarLite(map=map,
                    s_start=start,
                    s_goal=goal)

# SLAM to detect vertices
slam = SLAM(map=map,
            view_range=view_range)


# move and compute path
path, g, rhs = dstar.move_and_replan(robot_position=new_position)

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# import necessary sensors for go-to-task 
gps = robot.getDevice('gps')
gps.enable(timestep)
inertia = robot.getDevice("inertial unit")
inertia.enable(timestep)
camera = robot.getDevice("camera")

# set up sensors that robot will use 
motor = robot.getDevice('motor')
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
 
coordinates = path
j = 0

# example of a goal pose (if you are using a cell, would need to find pos within that cell) 
example_goal_posex, goal_posey = coordinates[0] #change to first index of the route list
goal_reached = False 
forward_speed = 5 

# Define the grid parameters
GRID_SIZE = 100  # Size of the grid (e.g., 100x100)
STEP_SIZE = 1.0  # Step size for discretization
OBSTACLE_COST = float('inf')  # Cost for grid cells with obstacles
GOAL_TOLERANCE = 0.5  # Tolerance for reaching the goal
   
start = (0, 0)
goal = (10, 10)
#static obstalces
obstacles = [(2, 2), (3, 3), (4, 4), (5, 5)]  # Example obstacle positions
obj_detected = False 

# function to help with movement 
def begin_rotating():
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(-0.2)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(0.2)
    
def move_forward():
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(forward_speed)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(forward_speed)
    
def move_back():
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(-forward_speed)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(-forward_speed)
    
def stop():
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(0)


i = 0 
j = 0 
while robot.step(timestep) != -1:
    roll, pitch, yaw = inertia.getRollPitchYaw()
    yaw = round(yaw, 2)
    robot_current_posx, robot_current_posy  = float(gps.getValues()[0]), float(gps.getValues()[1])

    if i == 0: 
        print(f'robot is starting from {robot_current_posx, robot_current_posy} to goal pose {example_goal_posex, goal_posey}')
        path = path, g, rhs = dstar.move_and_replan(robot_position=new_position)

    if path:
        print("Path found:", path)
        if obj_detected:
            new_observation = {"pos": (2,2), "type": 255} # 255 for obstacle, 0 for not
            # example: update position of obstacles 
            dstar.sensed_map.set_obstacle(pos=new_observation["pos"])

            # example: remove obstacle from a certain spot 
            dstar.sensed_map.remove_obstacle(pos=new_observation["pos"])
            
            # slam
            new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

            dstar.new_edges_and_old_costs = new_edges_and_old_costs
            dstar.sensed_map = slam_map

            # d star
            path, g, rhs = dstar.move_and_replan(robot_position=new_position)
            j = 0
            example_goal_posex, goal_posey = path[j]

            if not goal_reached:
                robot_current_posx, robot_current_posy  = float(gps.getValues()[0]), float(gps.getValues()[1])
            
    
        if math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) > 0.05 and yaw != round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2): 
         
            chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 

        elif math.dist([robot_current_posx, robot_current_posy], [path[-1][0], path[-1][0]]) <= 0.05:

            stop()
            goal_reached = True
            print(f'goal reached')

        elif math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) < 0.05 and j+1 < len(path):
            j += 1 
            example_goal_posex, goal_posey = path[j]
  

        if yaw != chosen_direction and not goal_reached: 
            begin_rotating()
                    
        elif yaw == chosen_direction and not goal_reached: 
            move_forward() 
            
    else:
        print("No path found.")
    
    i += 1
    pass

# Enter here exit cleanup code.
