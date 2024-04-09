from controller import Supervisor, Node, Keyboard, Emitter, Receiver, Field
import math 
import random
import ast
from math import pi

# set-up robot 
TIME_STEP = 32
robot = Supervisor()  # create Supervisor instance
emitter = robot.getDevice("emitter")
emitter.setChannel(2)
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(1) 

trials = 1
simulation_time = 600 
other_obstacle_ids = []

def message_listener(time_step):

    if receiver.getQueueLength()>0:
        # message = receiver.getData().decode('utf-8')
        # print('supervisor msgs --', message) 
        message = receiver.getString()
        
        if message: 
            receiver.nextPacket() 
        else: 
            receiver.nextPacket() 
            
    
# runs simulation for designated amount of time 
def run_seconds(t):
    
    n = TIME_STEP / 1000*32 # convert ms to s 
    start = robot.getTime()
    new_t = round(t, 1)
    
    while robot.step(TIME_STEP) != -1:
        # run robot simulation for 30 seconds (if t = 30)
        increments = TIME_STEP / 1000
        
        if robot.getTime() - start > new_t: 
            break 
    return 
    
def set_agent_up(start): 
    x, y = start 
    node = robot.getFromDef('main-e-puck')
    translation_field = node.getField('translation')
    new_value = [x, y, 0]
    translation_field.setSFVec3f(new_value)
    

def create_goal_msg(start = (0,0), goal = (0.3,0.3)):
    # update agent pos here 
    set_agent_up(start)
    
    msg = "goal_update:"
    startx, starty = start 
    goalx, goaly = goal 
    
    msg += str(startx) + "-" + str(starty) + "-"
    msg += str(goalx) +  "-" + str(goaly)
    
    print(f'attempting to send goal to controller: {msg}')
    
    emitter.send(msg)

def find_obstacles(curr_agent, obstacle_poses, range):
    poses_within_range = []
    for a in other_obstacle_ids:
        node = robot.getFromDef(str(a))
        pose = node.getPosition()
        distance = ((pose[0] - curr_agent[0])**2 + (pose[1] - curr_agent[1])**2)**0.5
        if distance <= range:
            poses_within_range.append(pose)
    return poses_within_range     
           
def run_optimization():
    
    for i in range(trials): 
        print('beginning new trial', i)
        
        # Update start and goal pose for given agent 
        create_goal_msg() # TODO: enable abiltiy to add custom start and goal pose 
        
        # time limit to run 
        run_seconds(simulation_time) 
            
        # include metric info here 
        msg = 'trial' + str(i)
        emitter.send(msg) 
        prev_msg = msg
        
    # reset env here 
         
    return 
  
def main(): 
    run_optimization()
    # save_progress()
         
main()
                    

