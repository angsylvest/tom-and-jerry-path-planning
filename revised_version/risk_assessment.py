import math
import numpy as np 

# assuming there is an observation period of about 10 seconds 

"""
Example input: 
Robot poses = [(x1, y2), ...]
Robot vel = 5.0 

Obstacle poses = [(x1, y2), ...]
Obstacle vel = 5.0

"""

class ObstacleAssessment():
    def __init__(self, robot_poses, robot_goal, obstacle_poses, obs_orient, robot_vel=5, obstacle_vel = 5):
        self.robot_poses = robot_poses
        self.robot_goal = robot_goal
        self.obstacle_poses = obstacle_poses
        self.obstacle_orient = obs_orient
        self.robot_vel = robot_vel
        self.obstacle_vel = obstacle_vel

        self.num_sectors = 8 
        self.counts = [0 for i in range(self.num_sectors)]
        self.total_counts = [0 for i in range(self.num_sectors)]

        self.x_train = []
        self.y_train = []

    def calculate_collision_time(self):
        agent_velocity = self.robot_vel
        obstacle_velocity = self.obstacle_vel

        if (agent_velocity > 0 and obstacle_velocity > 0) or (agent_velocity < 0 and obstacle_velocity < 0):
            return math.inf
        elif agent_velocity == 0 or obstacle_velocity == 0:
            return math.inf
        else:
            time_to_collision = abs(agent_velocity) / abs(obstacle_velocity)
            return time_to_collision
        

    def update_counts(self):
        num_collide = 0 
        for i in range(len(self.obstacle_poses)):
            # determine if possible to collide 
            collide = 'no'
            collision_time = self.calculate_collision_time() # using default val
            if collision_time != math.inf: 
                ttg = self.time_to_goal(self.robot_poses[i], self.robot_goal)
                if ttg <= collision_time: 
                    # id sector 
                    deg = math.degrees(self.obstacle_orient[i])
                    rang = self.sort_into_ranges(deg)
                    self.counts[rang] += 1
                    collide = 'yes'
                    num_collide += 1

            self.x_train.append(self.counts)
            self.y_train.append(collide)

        # run test of most current version 
        # might just need general prob based on observations
        prob = num_collide / len(self.obstacle_poses)

        return np.array(self.x_train), np.array(self.y_train), prob

    def distance(self, pos1, pos2): 
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def time_to_goal(self, current_pos, goal_pos):
        velocity = self.robot_vel
        return self.distance(current_pos, goal_pos) / velocity
    
    def compare_times(self, your_pos, your_velocity, obstacles_pos, obstacles_velocity, goal_pos, radians):
        
        your_time = self.time_to_goal(your_pos, goal_pos, your_velocity)
        obstacles_times = [self.time_to_goal(obs_pos, goal_pos, obs_velocity) for obs_pos, obs_velocity in zip(obstacles_pos, obstacles_velocity)]
        min_obstacle_time = min(obstacles_times)
        
        print("Your time to reach the goal:", your_time)
        print("Minimum obstacle time to reach the goal:", min_obstacle_time)
        
        if your_time < min_obstacle_time:
            print("You will reach the goal before any obstacle.")
            return 0.2
        elif your_time == min_obstacle_time:
            print("You and an obstacle will reach the goal at the same time.")
            return 0.8
        else:
            print("An obstacle will reach the goal before you.")
            return 
    
    def sort_into_ranges(self, angle_degrees):
        # Define the ranges
        ranges = [(0, 45), (45, 90), (90, 135), (135, 180),
                (180, 225), (225, 270), (270, 315), (315, 360)]
        
        # Iterate through the ranges and find where the angle falls
        for i, (start, end) in enumerate(ranges):
            if start <= angle_degrees < end:
                return i
        
        # If the angle is 360, include it in the last range
        if angle_degrees == 360:
            return len(ranges) - 1
        
        # If the angle is less than 0, include it in the first range
        if angle_degrees < 0:
            return 0
        
        return None

