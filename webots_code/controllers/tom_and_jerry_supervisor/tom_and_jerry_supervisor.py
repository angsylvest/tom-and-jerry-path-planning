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
simulation_time = 6000 

obst_def = ["obst-1"]

def message_listener():

    if receiver.getQueueLength()>0:
        # message = receiver.getData().decode('utf-8')
        # print('supervisor msgs --', message) 
        message = receiver.getString()
        
        if 'obj-detected' in message: 
            curr_node = robot.getFromDef('main-e-puck')
            curr_x, curr_y, curr_z = curr_node.getField('translation').getSFVec3f()
            curr_yaw = curr_node.getField('rotation').getSFVec3f()

            obt_node = robot.getFromDef('obst-1')
            obt_x, obt_y, obt_z = obt_node.getField('translation').getSFVec3f()
            print(f'obstacle pose: {obt_x, obt_y}')
            obt_yaw = obt_node.getField('rotation').getSFVec3f()

            msg = "obj-info|"

            # include more prior knowledge .. (depending on if dynamic or static obst)
            rob_poses = [(round(curr_x,2), round(curr_y,2))]
            obst_poses = [(round(obt_x,2), round(obt_y,2))]
            obs_orient = [round(obt_yaw[2],2)]

            # Add robot poses to the message
            msg += f"robot_pose={rob_poses}|"

            # Add obstacle poses to the message
            msg += f"obstacle_pose={obst_poses}|"

            # Add obstacle orientations to the message
            msg += f"obstacle_orientation={obs_orient}|"

            emitter.send(msg)

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
        message_listener()
        if robot.getTime() - start > new_t: 
            break 
    return 
    
def set_agent_up(start): 
    x, y = start 
    node = robot.getFromDef('main-e-puck')
    translation_field = node.getField('translation')

    # print(f'original info: {translation_field.getSFVec3f()}')
    new_value = [x, y, 0]
    translation_field.setSFVec3f(new_value)
    

def create_goal_msg(start = (0,0), goal = (0.115,0.42)):
    # update agent pos here 
    set_agent_up(start)
    
    msg = "goal_update:"
    startx, starty = start 
    goalx, goaly = goal 
    
    msg += str(startx) + "|" + str(starty) + "|"
    msg += str(goalx) +  "|" + str(goaly)
    
    print(f'attempting to send goal to controller: {msg}')
    
    emitter.send(msg)
     
           
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
                    

