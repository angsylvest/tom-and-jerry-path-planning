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

# include relevant import statements regarding path generation here.. 
import sys 
sys.path.append('../../../')

from revised_version.tom_and_jerry_generator import * 
from revised_version.grid_environment import * 
from revised_version.evolving_waypoint import * 

env = TomAndJerryEnvironment(dims=(1,1), upper_left=(-0.5, 0.5), grid_dim=0.2)
path_generator = TomAndJerry(env, current_pos=((0, 0)), goal_pos=(0.4, 0.4))

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
emitter = robot.getDevice("emitter")
emitter.setChannel(1)
receiver = robot.getDevice("receiver") 
receiver.enable(timestep)
receiver.setChannel(2) 
ds = robot.getDevice('distance sensor') # front 
ds.enable(timestep)

# set up sensors that robot will use 
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

start = (0, 0)
goal = (10, 10)
#static obstalces
obstacles = [(2, 2), (3, 3), (4, 4), (5, 5)]  # Example obstacle positions
obj_detected = False 

coordinates = path_generator.a_star_path()
j = 0

# example of a goal pose (if you are using a cell, would need to find pos within that cell) 
 #change to first index of the route list
goal_reached = False 
forward_speed = 5 

# Define the grid parameters
GRID_SIZE = 100  # Size of the grid (e.g., 100x100)
STEP_SIZE = 1.0  # Step size for discretization
OBSTACLE_COST = float('inf')  # Cost for grid cells with obstacles
GOAL_TOLERANCE = 0.5  # Tolerance for reaching the goal
   

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
    

    if receiver.getQueueLength()>0:
        message = receiver.getString()
        print(f'current message to taj: {message}')
        
        if 'goal_update' in message: 
            msg = message.split(':')[1].split('-')
            startx = float(msg[0])
            starty = float(msg[1])
            
            goalx = float(msg[2])
            goaly = float(msg[3])
            
            start = startx, starty
            goal = goalx, goaly 
            example_goal_posex, goal_posey = goal
            
            print(f'inputs x and y {startx, starty} for goals {goalx, goaly}')
            
            path_generator = TomAndJerry(env, current_pos=(startx, starty), goal_pos=(goalx, goaly))
            coordinates = path_generator.a_star_path()
            
            j = 0
            
            print(f'updated path: {coordinates}')
            
            receiver.nextPacket() 
            
        elif 'obj-info' in message: 
            evolving_waypoint_index = 0
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
            currx = round(currx, 2)
            curry = round(curry, 2)
            evolving_waypoint_index += 1 
            evolving = True 
            
        else: 
            receiver.nextPacket() 

#upper left is the one on Webots
result = gen_grid((-0.379201, 0.359201), 4, 4, 1) 
# Main loop:
# - perform simulation steps until Webots is stopping the controller

evolving_waypoint_index = 0 
evolving = False 
path_index = 0 

i = 0 
while robot.step(timestep) != -1:
    message_listener()
    
    dist_val = ds.getValue()
    if dist_val < 1000: 
        obj_detected = True 
    
    roll, pitch, yaw = inertia.getRollPitchYaw()
    yaw = round(yaw, 2)
    if not goal_reached:
        robot_current_posx, robot_current_posy  = float(gps.getValues()[0]), float(gps.getValues()[1])

    # initially create path from A* 
    if i == 0: 
        print(f'robot is starting from {robot_current_posx, robot_current_posy} to goal pose {example_goal_posex, goal_posey}')
        path = path_generator.a_star_path()

    if path:
        # print("Path found:", path)
        if obj_detected and not evolving: # not actually using obstacle data yet .. 
            # calc fid + evolving waypoints 
            # request supervisor to send info about obstacle
            msg = "obj-detected"
            emitter.send(msg)
    
        if math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) > 0.05 and yaw != round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2): 
         
            chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 

        elif math.dist([robot_current_posx, robot_current_posy], [path[-1][0], path[-1][0]]) <= 0.05:
            print(f'goal successfully reached') 
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
                    goal = goalx, goaly 
                    path_generator = TomAndJerry(env, current_pos=(robot_current_posx, robot_current_posy), goal_pos=(goalx, goaly))
                    # update to be new path 
                    path = path_generator.a_star_path()
                    j = 0 
                    example_goal_posex, goal_posey = path[0]
                    obj_detected = False 
  
        if yaw != chosen_direction and not goal_reached: 
            begin_rotating()
                    
        elif yaw == chosen_direction and not goal_reached: 
            move_forward() 
            
    else:
        print("No path found.")
    
    i += 1
    pass

# Enter here exit cleanup code.
