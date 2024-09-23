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

# include relevant import statements regarding path generation here.. 
import sys 
sys.path.append('../../../')

from revised_version.tom_and_jerry_generator import * 
from revised_version.grid_environment import * 
from revised_version.evolving_waypoint import * 
from revised_version.utility_func import * 

env = TomAndJerryEnvironment(dims=(1,1), upper_left=(-0.5, 0.5), grid_dim=0.2)
path_generator = TomAndJerry(env, current_pos=(0, 0), goal_pos=(0.4, 0.4))

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


obj_detected = False 
coordinates = path_generator.a_star_path()
j = 0

# example of a goal pose (if you are using a cell, would need to find pos within that cell) 
 #change to first index of the route list
goal_reached = False 
forward_speed = 3 
moving_forward = False 
time_forward = 0 

# Define the grid parameters
GRID_SIZE = 100  # Size of the grid (e.g., 100x100)
STEP_SIZE = 1.0  # Step size for discretization
OBSTACLE_COST = float('inf')  # Cost for grid cells with obstacles
GOAL_TOLERANCE = 0.5  # Tolerance for reaching the goal
   

# function to help with movement 
def begin_rotating():
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(-0.1)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(0.1)
    
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
            
            print(f'inputs x and y {startx, starty} for goals {goalx, goaly}')
            
            path_generator = TomAndJerry(env, current_pos=(startx, starty), goal_pos=(goalx, goaly))
            coordinates = path_generator.a_star_path()
            j = 0
            example_goal_posex, goal_posey = coordinates[j]
            
            
            print(f'updated path: {coordinates}')
            
            receiver.nextPacket() 
            
        elif 'obj-info' in message: 
            # unpack obs info 

            print(f'info from msg: {message}')
            msg = message.split("|")

            rob_poses = ast.literal_eval(msg[1].split("=")[1]) # [(0,0), (0,0), (0,0), (0,0), (0,0)]
            obst_poses = ast.literal_eval(msg[2].split("=")[1]) # [(1,0), (1,0), (1,0), (1,0), (1,0)]
            obs_orient = ast.literal_eval(msg[3].split("=")[1])  #[0.785398, 1.5708, 2.35619, 3.14159, 3.92699]
            risk_assessment = ObstacleAssessment(robot_poses=rob_poses, robot_goal=goal, obstacle_poses=obst_poses, obs_orient=obs_orient, curr_path=coordinates, obstacle_vel=0)
            
            x_train, y_train, prob = risk_assessment.update_counts()
            curr_pose = float(gps.getValues()[0]), float(gps.getValues()[1])
            radius = 0.35 # radius of e-puck robot

            evolving_waypoint_index = 0
            
            obs_pose = obst_poses[-1] # just example, can be changed .. 
            sector_dict, dist, ait, opt_rad = path_generator.calc_fid(path, obs_pose, obs_orient, prob, curr_pose, obs_pose) # inputs: current path, 
            print(f'obs pose: {obs_pose} for optimized radius: {opt_rad} and curr_pose for agent {curr_pose} and goal: {goal}')
            # tangent_start, tangent_end, marked_coordinates = get_circle_paths_and_coordinates(obs_pose, opt_rad, curr_pose, goal) # FID radius, center is where obstacle is 
            marked_coordinates, curve_points = path_generator.better_generate_waypoints(coordinates, obst_poses[-1], opt_rad)
            print(f'updated marked coords: {marked_coordinates}')
            currx, curry = marked_coordinates[evolving_waypoint_index]
            currx = round(currx, 2)
            curry = round(curry, 2)
            example_goal_posex, goal_posey = currx, curry
            evolving_waypoint_index += 1 
            evolving = True

            receiver.nextPacket()  
            
        else: 
            receiver.nextPacket() 

#upper left is the one on Webots
result = gen_grid((-0.379201, 0.359201), 4, 4, 1) 
# Main loop:
# - perform simulation steps until Webots is stopping the controller

evolving_waypoint_index = 0 
evolving = False 
path_index = 0 
delayed = False
time_since_delay = 0 

i = 0 
while robot.step(timestep) != -1:
    message_listener()
    
    dist_val = ds.getValue()
    if dist_val < 500:  
        if not delayed: 
            obj_detected = True 
            delayed = True
            print('obj detected..')
        elif time_since_delay > 100: 
            time_since_delay = 0 
            delayed = False 
            obj_detected = False

    
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
    
        if math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) > 0.18 and yaw != round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2): 
            # print(f'large dist: {math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey])}')
            chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 


        elif math.dist([robot_current_posx, robot_current_posy], [path[-1][0], path[-1][0]]) <= 0.03:
            time_to_goal = robot.getTime() - initial_time 
            print(f'goal successfully reached in time: {time_to_goal}') 
    
            stop()
            goal_reached = True
            break 
     
        else: 
            # Emily: here it stops, but you can just update the goal pose here if you have a list
            if j + 1 < len(coordinates) and not evolving:
                if math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) < 0.15:
                    example_goal_posex, goal_posey = coordinates[j + 1]
                    chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 
                    goal_reached = False
                    j+= 1
                    print('moving onto next pos')
                
            elif evolving: 
                if not obj_detected: 
                    evolving_waypoint_index = len(marked_coordinates) + 1 

                if evolving_waypoint_index + 1 < len(marked_coordinates): 
                    if math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) < 0.15:
                        # example_goal_posex, goal_posey = coordinates[evolving_waypoint_index + 1]
                        example_goal_posex, goal_posey = marked_coordinates[evolving_waypoint_index + 1]
                        chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 
                        goal_reached = False
                        # j+= 1
                        print('moving onto next pos')
                
                        # example_goal_posex, goal_posey = marked_coordinates[evolving_waypoint_index]
                        evolving_waypoint_index += 1 
                        print(f'updated pos bc of evolving waypoint: {evolving_waypoint_index, len(marked_coordinates)}')
                else: 
                    evolving = False 
                    goalx, goaly = goal 
                    print(f'regenerating a start: {robot_current_posx, robot_current_posy} with goal {goalx, goaly}')
                    path_generator = TomAndJerry(env, current_pos=(robot_current_posx, robot_current_posy), goal_pos=(goalx, goaly))
                    # update to be new path 
                    path = path_generator.a_star_path()
                    j = 0 
                    example_goal_posex, goal_posey = path[0]
                    chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 
                    obj_detected = False 

        # print(f' mvoing forward: {moving_forward} and time forward amount : {time_forward}')
        if moving_forward:  
            if time_forward > 100: 
                print('able to to change to rotating state')
                time_forward = 0 
                moving_forward = False 
            else: 
                time_forward += 1

        if yaw != chosen_direction and not goal_reached and not moving_forward: 
            stop()
            begin_rotating()
                    
        elif yaw == chosen_direction and not goal_reached: 
            stop()
            move_forward() 
            

        # print(f'dist from goal: {example_goal_posex, goal_posey}: {math.dist([robot_current_posx, robot_current_posy], [path[-1][0], path[-1][0]])} with chosen direction: {chosen_direction} and curr orientation: {yaw}')
        
    else:
        print("No path found.")
    
    i += 1
    pass

# Enter here exit cleanup code.
