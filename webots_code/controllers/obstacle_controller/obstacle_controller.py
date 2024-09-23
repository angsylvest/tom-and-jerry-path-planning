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
 
goal_reached = False 
forward_speed = 5 

# Define the grid parameters
GRID_SIZE = 100  # Size of the grid (e.g., 100x100)
STEP_SIZE = 1.0  # Step size for discretization
OBSTACLE_COST = float('inf')  # Cost for grid cells with obstacles
GOAL_TOLERANCE = 0.5  # Tolerance for reaching the goal
 
type_of_obstacle = ["static", "single_dyn", "multi_dyn"]
type_index = 0

# Initialize an empty list to store the coordinates
coordinates = []

# Open the paths.txt file for reading

if type_index != 0: 
    with open('paths.txt', 'r') as file:
        # Read each line in the file
        for line in file:
            # Strip any leading or trailing whitespace characters
            line = line.strip()
            # Split the line into x and y coordinates
            x, y = map(float, line.strip('()').split(','))
            # Append the coordinates as a tuple to the list
            coordinates.append((x, y))
else: 
    robot_current_posx, robot_current_posy  = float(gps.getValues()[0]), float(gps.getValues()[1])
    coordinates.append((robot_current_posx, robot_current_posy))

example_goal_posex, goal_posey = coordinates[0] 
path = coordinates

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
        chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 

    if path:
        # print("Path found:", path)
    
        if math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) > 0.05 and yaw != round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2): 
         
            chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 

        elif math.dist([robot_current_posx, robot_current_posy], [path[-1][0], path[-1][0]]) <= 0.05:

            if type_index == 1: 
                stop()
                goal_reached = True
                print(f'goal reached')
            if type_index == 2: 
                j = 0 
            

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
