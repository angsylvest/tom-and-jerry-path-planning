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

from revised_version.tom_and_jerry_generator import * 
from revised_version.grid_environment import * 
from revised_version.evolving_waypoint import * 

env = TomAndJerryEnvironment(dims=(1,1), upper_left=(-0.5, 0.5), grid_dim=0.2)
path_generator = TomAndJerry(env, current_pos=((0, 0)), goal_pos=(0.4, 0.4))

# include relevant import statements regarding path generation here.. 

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

evolving_waypoint_index = 0 
evolving = False 
path_index = 0 

i = 0 
while robot.step(timestep) != -1:
    roll, pitch, yaw = inertia.getRollPitchYaw()
    yaw = round(yaw, 2)

    # initially create path from A* 
    if i == 0: 
        print(f'robot is starting from {robot_current_posx, robot_current_posy} to goal pose {example_goal_posex, goal_posey}')
        path = path_generator.a_star_path()

    if path:
        print("Path found:", path)
        if obj_detected:
            # calc fid + evolving waypoints 
            radians = [0.785398, 1.5708, 2.35619, 3.14159, 3.92699, 4.71239, 5.49779, 6.28319]
            X_train = np.array([[0, 3, 4, 0, 2],
                            [0, 1, 4, 0, 2],
                            [0, 3, 4, 0, 2],
                            [0, 3, 4, 0, 2],
                            [0, 3, 4, 0, 2]])
            y_train = np.array(['yes', 'no', 'yes', 'yes', 'no'])
            X_test = np.array([[0, 3, 4, 0, 2]])

            obs_pose = (3,3) # just example, can be changed .. 
            sector_dict, dist = path_generator.calc_fid(path, (0.2, 0.4), radians, X_train, y_train, X_test, start_pos, obs_pose) # inputs: current path, 
            tangent_start, tangent_end, marked_coordinates = get_circle_paths_and_coordinates(obs_pose, dist, start_pos, goal_pos) # FID radius, center is where obstacle is 
            currx, curry = marked_coordinates[evolving_waypoint_index]
            evolving_waypoint_index += 1 
            evolving = True 
            if distance < sector_dict:
                move_forward()


            if not goal_reached:
                robot_current_posx, robot_current_posy  = float(gps.getValues()[0]), float(gps.getValues()[1])
            
    
        if math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) > 0.05 and yaw != round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2): 
         
            chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 

        elif math.dist([robot_current_posx, robot_current_posy], [path[-1][0], path[-1][0]]) <= 0.05:

            stop()
            goal_reached = True
     

        else: 
            # Emily: here it stops, but you can just update the goal pose here if you have a list
            if j + 1 < len(coordinates) and not evolving:
                example_goal_posex, goal_posey = coordinates[j + 1]
                goal_reached = False
                j+= 1
            elif evolving: 
                if evolving_waypoint_index + 1 < len(marked_coordinates): 
                    example_goal_posex, goal_posey = marked_coordinates[evolving_waypoint_index]
                    evolving_waypoint_index += 1 
                else: 
                    evolving = False 
                    path = path_generator.a_star_path()
                    j = 0 
                    example_goal_posex, goal_posey = path[0]
  
        if yaw != chosen_direction and not goal_reached: 
            begin_rotating()
                    
        elif yaw == chosen_direction and not goal_reached: 
            move_forward() 
            
    else:
        print("No path found.")
    
    i += 1
    pass

# Enter here exit cleanup code.
