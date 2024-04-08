# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, InertialUnit, GPS, DistanceSensor
import math 
from math import sin, cos, pi 

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# import necessary sensors for go-to-task 
gps = robot.getDevice('gps')
gps.enable(timestep)
inertia = robot.getDevice("inertial unit")
inertia.enable(timestep)

# set up sensors that robot will use 
motor = robot.getDevice('motor')
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
 

coordinates = [(-0.22, -0.15)]
j = 0

# example of a goal pose (if you are using a cell, would need to find pos within that cell) 
example_goal_posex, goal_posey = coordinates[0] #change to first index of the route list
goal_reached = False 
forward_speed = 5

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

# Main loop:
# - perform simulation steps until Webots is stopping the controller
i = 0 
while robot.step(timestep) != -1:

    if not goal_reached:
    
        robot_current_posx, robot_current_posy  = float(gps.getValues()[0]), float(gps.getValues()[1])
        if i == 0: 
            print(f'robot is starting from {robot_current_posx, robot_current_posy} to goal pose {example_goal_posex, goal_posey}')
    
        # print(f'curr distance away {math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey])}')
        if math.dist([robot_current_posx, robot_current_posy], [example_goal_posex, goal_posey]) > 0.05: 
         
            chosen_direction = round(math.atan2(goal_posey-robot_current_posy,example_goal_posex-robot_current_posx),2) 
     
        else: 
            if not goal_reached: 
                goal_reached = True 
                print(f'agent has reached its goal: {robot_current_posx, robot_current_posy}') 
                
    
                # Emily: here it stops, but you can just update the goal pose here if you have a list
            if j + 1 < len(coordinates):
                example_goal_posex, goal_posey = coordinates[j + 1]
                goal_reached = False
                j+= 1
            else:
                stop()
                goal_reached = True
                # stop()
                
            
            #if else statement for final goal reached otherwise keep moving in next postion
                #include if goal is not reached move to next index on path, place within if else
                #if obstacle in range; run tom and jerry algorithm
    # if not goal_reached: 
        roll, pitch, yaw = inertia.getRollPitchYaw()
        yaw = round(yaw, 2)
        # print("Orientation", yaw)
        print(chosen_direction)
                
        if yaw != chosen_direction and not goal_reached: 
            begin_rotating()
                    
        elif yaw == chosen_direction and not goal_reached: 
            move_forward() 
   
    
    i += 1
    pass

# Enter here exit cleanup code.
