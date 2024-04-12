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

from baselines.lifelong_a_star import * 
# using bc already has an occupancy grid 
from baselines.simplified_d_star_lite.grid import * 
from revised_version.utility_func import *


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
emitter = robot.getDevice("emitter")
emitter.setChannel(1)
receiver = robot.getDevice("receiver") 
receiver.enable(timestep)
receiver.setChannel(2) 
 
# coordinates = path
j = 0

# example of a goal pose (if you are using a cell, would need to find pos within that cell) 
# example_goal_posex, goal_posey = coordinates[0] #change to first index of the route list
goal_reached = False 
forward_speed = 5 

# Define the grid parameters
# GRID_SIZE = 100  # Size of the grid (e.g., 100x100)
STEP_SIZE = 1.0  # Step size for discretization
OBSTACLE_COST = float('inf')  # Cost for grid cells with obstacles
GOAL_TOLERANCE = 0.5  # Tolerance for reaching the goal
   
# include relevant import statements regarding path generation here.. 
GRID_SIZE = 0.2
ENV_LENGTH = 1
x_dim =  ENV_LENGTH // GRID_SIZE
y_dim = ENV_LENGTH // GRID_SIZE

# start and goal pos from actual env
start = (0, 0)
goal = (0.5, 0.5)

map = OccupancyGridMap(x_dim=x_dim,
                            y_dim=y_dim,
                            exploration_setting='4N')

#static obstalces
obstacles = [(2, 2), (3, 3), (4, 4), (5, 5)]  # Example obstacle positions
obj_detected = False 

grid_rep = list(map)
path = LifelongAStar(grid_rep).lifelong_astar(start, goal)
example_goal_posex, goal_posey = path[j]

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


def message_listener():
    global goal 
    global start 
    global example_goal_posex
    global goal_posey
    global j 
    global path_generator 
    global coordinates
    global evolving_waypoint_index
    global evolving
    global marked_coordinates
    

    if receiver.getQueueLength()>0:
        message = receiver.getString()
        print(f'current message to taj: {message}')
        
        if 'goal_update' in message: 
            msg = message.split(':')[1].split('|')
            startx = float(msg[0])
            starty = float(msg[1])
            
            goalx = float(msg[2])
            goaly = float(msg[3])
            
            start = startx, starty
            goal = goalx, goaly 
            example_goal_posex, goal_posey = goal

            map = OccupancyGridMap(x_dim=x_dim,
                                        y_dim=y_dim,
                                        exploration_setting='4N')

            #static obstalces
            obstacles = [(2, 2), (3, 3), (4, 4), (5, 5)]  # Example obstacle positions
            obj_detected = False 

            grid_rep = list(map)
            path = LifelongAStar(grid_rep).lifelong_astar(start, goal)
            example_goal_posex, goal_posey = path[j]
            
            receiver.nextPacket() 
            
        elif 'obj-info' in message: 
            # unpack obs info 
            print(f'info from msg: {message}')
            msg = message.split("|")

            obs_pose = msg 
            grid_obsx, grid_obsy = real_to_grid_pos(real_pos=obs_pose, env_size=(ENV_LENGTH,ENV_LENGTH), upper_left_corner=(-0.5, 0.5), grid_size = GRID_SIZE)

            map = OccupancyGridMap(x_dim=x_dim,
                                        y_dim=y_dim,
                                        exploration_setting='4N')

            #static obstalces
            # obstacles = [(2, 2), (3, 3), (4, 4), (5, 5)]  # Example obstacle positions
            # obj_detected = False 

            grid_rep = list(map)

            # put obs_pose here .. 

            
            path = LifelongAStar(grid_rep).lifelong_astar(start, goal)
            example_goal_posex, goal_posey = path[j]

            j = 0
            example_goal_posex, goal_posey = path[j]

            receiver.nextPacket()  
            
        else: 
            receiver.nextPacket() 
    

#node descrtization
def gen_grid(upper_left, num_rows, num_cols, dist):
    upperx, uppery = upper_left 
    Grid_pos = [(upperx, uppery)]
    for i in range(num_cols):
        for j in range(num_rows):
            Next_center = (upperx + (i * dist), uppery - (j * dist))
            Grid_pos.append(Next_center)
    return Grid_pos

#upper left is the one on Webots
result = gen_grid((-0.379201, 0.359201), 4, 4, 1) 
# Main loop:
# - perform simulation steps until Webots is stopping the controller


i = 0 
while robot.step(timestep) != -1:
    roll, pitch, yaw = inertia.getRollPitchYaw()
    yaw = round(yaw, 2)
    robot_current_posx, robot_current_posy  = float(gps.getValues()[0]), float(gps.getValues()[1])

    if i == 0: 
        print(f'robot is starting from {robot_current_posx, robot_current_posy} to goal pose {example_goal_posex, goal_posey}')
        path = LifelongAStar(grid_rep).lifelong_astar(start, goal)

    if path:
        print("Path found:", path)
        if obj_detected:
            msg = "obj-detected"
            emitter.send(msg)
    
        if math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) > 0.05 and yaw != round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2): 
         
            chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 

        elif math.dist([robot_current_posx, robot_current_posy], [path[-1][0], path[-1][0]]) <= 0.05:

            stop()
            goal_reached = True

        else: 
            # Emily: here it stops, but you can just update the goal pose here if you have a list
            if j + 1 < len(path):
                example_goal_posex, goal_posey = path[j + 1]
                goal_reached = False
                j+= 1
     

        if yaw != chosen_direction and not goal_reached: 
            begin_rotating()
                    
        elif yaw == chosen_direction and not goal_reached: 
            move_forward() 
            
    else:
        print("No path found.")
    
    i += 1
    pass

# Enter here exit cleanup code.
