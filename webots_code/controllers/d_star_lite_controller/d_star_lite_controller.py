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
import ast

import sys 
sys.path.append('../../../')

from baselines.simplified_d_star_lite.grid import * 
from baselines.simplified_d_star_lite.utils import * 
from baselines.simplified_d_star_lite.d_star_lite import * 
from revised_version.utility_func import *

OBSTACLE = 255
UNOCCUPIED = 0

# include relevant import statements regarding path generation here.. 
GRID_SIZE = 0.2
ENV_LENGTH = 1
x_dim =  int(ENV_LENGTH / GRID_SIZE) + 1 
y_dim = int(ENV_LENGTH / GRID_SIZE) + 1

print(f'x_dim {x_dim} and y_dim {y_dim}')
# start and goal pos from actual env
start = (0, 0)
goal = (0.3, 0.3)
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
print(f'new map for d* lite looks like: {map.occupancy_grid_map} with start {startx, starty} and goal {goalx, goaly}')
dstar = DStarLite(map=map,
                    s_start=(startx, starty),
                    s_goal=(goalx, goaly))

# SLAM to detect vertices
slam = SLAM(map=map,
            view_range=view_range)


# move and compute path
path, g, rhs = dstar.move_and_replan(robot_position=new_position)
print(f'updated path from d star lite: {path}')

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
initial_time = robot.getTime()

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
ds = robot.getDevice('distance sensor') # front 
ds.enable(timestep)
 
coordinates = path
j = 0

# example of a goal pose (if you are using a cell, would need to find pos within that cell) 
# example_goal_posex, goal_posey = coordinates[0] #change to first index of the route list
example_goal_posex, goal_posey = grid_to_real_pos(grid_pos=coordinates[j])
goal_reached = False 
forward_speed = 5 

# Define the grid parameters
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

def message_listener():
    global goal 
    global start 
    global example_goal_posex
    global goal_posey
    global j 
    global coordinates
    global path
    global chosen_direction
    

    if receiver.getQueueLength()>0:
        message = receiver.getString()
        # print(f'current message to taj: {message}')
        
        if 'goal_update' in message: 
            msg = message.split(':')[1].split('|')
            startx = float(msg[0])
            starty = float(msg[1])
            
            goalx = float(msg[2])
            goaly = float(msg[3])

            start = startx, starty
            goal = goalx, goaly
            
            print(f'goal input: {goalx, goaly}')
            
            # need way to convert cur pos to grid space pos 
            startx, starty = real_to_grid_pos(real_pos=(startx, starty), env_size=(ENV_LENGTH,ENV_LENGTH), upper_left_corner=(-0.5, 0.5), grid_size = GRID_SIZE)
            goalx, goaly = real_to_grid_pos(real_pos=(goalx, goaly), env_size=(ENV_LENGTH,ENV_LENGTH), upper_left_corner=(-0.5, 0.5), grid_size = GRID_SIZE)

            # example_goal_posex, goal_posey = goal
            print(f'new map for d* lite looks like: {map.occupancy_grid_map} with grid goal {goalx, goaly}')
            dstar = DStarLite(map=map,
                                s_start=(startx, starty),
                                s_goal=(goalx, goaly))

            # SLAM to detect vertices
            slam = SLAM(map=map,
                        view_range=view_range)


            # move and compute path
            path, g, rhs = dstar.move_and_replan(robot_position=(startx, starty))
            j = 0 

            # example_goal_posex, goal_posey = path[j]
            example_goal_posex, goal_posey = grid_to_real_pos(grid_pos=path[j])
            chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 
            
            print(f'updated path: {path} with next goal pos {example_goal_posex, goal_posey}')
            
            receiver.nextPacket() 
            
        elif 'obj-info' in message: 
            # unpack obs info 
            msg = message.split("|")

            rob_poses = ast.literal_eval(msg[1].split("=")[1]) # [(0,0), (0,0), (0,0), (0,0), (0,0)]
            obst_poses = ast.literal_eval(msg[2].split("=")[1]) # [(1,0), (1,0), (1,0), (1,0), (1,0)]
            obs_orient = ast.literal_eval(msg[3].split("=")[1])  #[0.785398, 1.5708, 2.35619, 3.14159, 3.92699]

            # need way to convert cur pos to grid space pos 
            obsx, obsy = real_to_grid_pos(real_pos=obst_poses[0], env_size=(ENV_LENGTH,ENV_LENGTH), upper_left_corner=(-0.5, 0.5), grid_size = GRID_SIZE)

            new_observation = {"pos": (obsx, obsy), "type": 255} 
            startx, starty = (robot_current_posx, robot_current_posy)
            start = real_to_grid_pos(real_pos=(startx, starty), env_size=(ENV_LENGTH,ENV_LENGTH), upper_left_corner=(-0.5, 0.5), grid_size = GRID_SIZE)
            goalx, goaly = real_to_grid_pos(real_pos=goal, env_size=(ENV_LENGTH,ENV_LENGTH), upper_left_corner=(-0.5, 0.5), grid_size = GRID_SIZE)
            
            # print(f'observation input: {new_observation} with start {startx, starty} ')

            dstar = DStarLite(map=map,
                    s_start=start,
                    s_goal=(goalx, goaly))

            # SLAM to detect vertices
            slam = SLAM(map=map,
                        view_range=view_range)
            
            # example: update position of obstacles 
            dstar.sensed_map.set_obstacle(pos=new_observation["pos"])

            # # example: remove obstacle from a certain spot  
            # dstar.sensed_map.remove_obstacle(pos=new_observation["pos"])
            
            new_position = start 
            
            # slam
            new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

            dstar.new_edges_and_old_costs = new_edges_and_old_costs
            dstar.sensed_map = slam_map

            # d star
            path, g, rhs = dstar.move_and_replan(robot_position=new_position)
            
            # print(f'path generated after obs detected: {path}')
            j = 0
            example_goal_posex, goal_posey = grid_to_real_pos(grid_pos=path[j])

            receiver.nextPacket()  
            
        else: 
            receiver.nextPacket() 


i = 0 
j = 0 
while robot.step(timestep) != -1:
    roll, pitch, yaw = inertia.getRollPitchYaw()
    yaw = round(yaw, 2)
    robot_current_posx, robot_current_posy  = float(gps.getValues()[0]), float(gps.getValues()[1])
    message_listener()
    
    dist_val = ds.getValue()
    if dist_val < 1000: 
        obj_detected = True 

    # if i == 0: 
        # print(f'robot is starting from {robot_current_posx, robot_current_posy} to goal pose {example_goal_posex, goal_posey}')
        # path = path, g, rhs = dstar.move_and_replan(robot_position=new_position)

    if path:
        # print("Path found:", path)
        # print(f'dist from goal: {math.dist([robot_current_posx, robot_current_posy], [goal[0], goal[1]])}')
        if obj_detected:

            msg = "obj-detected"
            emitter.send(msg)
    
        if math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) > 0.12 and yaw != round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2): 
         
            chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 
        

        elif math.dist([robot_current_posx, robot_current_posy], [goal[0], goal[1]]) <= 0.05:

            stop()
            goal_reached = True
            time_to_goal = robot.getTime() - initial_time 
            print(f'goal successfully reached in time: {time_to_goal}') 
            break

        elif math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) < 0.05 and j+1 < len(path):
            j += 1 
            # example_goal_posex, goal_posey = path[j]
            example_goal_posex, goal_posey = grid_to_real_pos(grid_pos=path[j])
  

        if yaw != chosen_direction and not goal_reached: 
            begin_rotating()
                    
        elif yaw == chosen_direction and not goal_reached: 
            move_forward() 
            
    else:
        print("No path found.")
    
    i += 1
    pass

# Enter here exit cleanup code.
