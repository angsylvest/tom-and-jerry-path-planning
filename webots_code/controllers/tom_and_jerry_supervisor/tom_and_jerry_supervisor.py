from controller import Supervisor, Node, Keyboard, Emitter, Receiver, Field
import math 
import random
import ast
from math import pi

# set-up robot 
TIME_STEP = 32
robot = Supervisor()  # create Supervisor instance
emitter = robot.getDevice("emitter")
emitter.setChannel(1)
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 

trials = 1
simulation_time = 60

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
   
           
def run_optimization():
    
    for i in range(trials): 
        print('beginning new trial', i)
        emitter.send(str(msg).encode('utf-8'))
        run_seconds(simulation_time) 
            
        # include metric info here 
        msg = 'trial' + str(i)
        emitter.send(msg.encode('utf-8')) 
        prev_msg = msg
        
    # reset env here 
         
    return 
  
def main(): 
    run_optimization()
    # save_progress()
         
main()
                    



# # set up environments 
# def generate_robot_central(num_robots):

#     global fitness_scores 
#     global collected_count 
#     global population
#     global columns 
#     global r_pos_to_generate
#     global pairs 
#     global overall_fitness_scores
#     global prev_msg 
#     global id_msg
#     global id_ind
    
#     global input_from_others
    
#     curr_msg = str("size-" + str(num_robots))
#     if curr_msg != prev_msg: 
#         emitter.send(str("size-" + str(num_robots)).encode('utf-8'))
#         prev_msg = curr_msg
    
#     if len(population) != 0: 
    
#         for r in population: 
#             r.remove()
             
#     population = []
#     fitness_scores = []
#     overall_fitness_scores = []
#     collected_count = []
#     pairs = []
#     id_msg = "ids"
#     id_ind = "id_ind"
        
#     for i in range(num_robots):
#         curr_key = f'agent-{i}'
#         rootNode = robot.getRoot()
#         rootChildrenField = rootNode.getField('children')
#         if i == 0: 
#             robot_name = "k0" 
#         else: 
#             robot_name = "k0(" + str(i) + ")"
        
#         # Import the khepera PROTO with a dynamically set robot name
#         import_string = 'khepera {{ robotName "{0}" }}'.format(robot_name)
#         rootChildrenField.importMFNodeFromString(-1, import_string)
        
#         rec_node = rootChildrenField.getMFNode(-1)
        
#         # rec_node = robot.getFromDef('khepera') 
#         t_field = rec_node.getField('translation')
       
#         input_from_others[curr_key] = 0
        
#         pose = [round(random.uniform(0.3, -0.3),2), round(random.uniform(0.3, -0.3) ,2), 0.02]
#         while pose in r_pos_to_generate: # remove any duplicates
#             pose = [round(random.uniform(0.3, -0.3),2), round(random.uniform(0.3, -0.3) ,2), 0.02]
#         r_pos_to_generate.append(pose)
        
#         t_field.setSFVec3f(pose)
        
#         # sets up metrics 
#         fitness_scores.append("!")
#         overall_fitness_scores.append('!')
#         pairs.append("!")
#         collected_count.append(0)
#         population.append(rec_node)
#         id_msg += " " + str(rec_node.getId()) 
#         id_ind += " " + curr_key