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
    def __init__(self, robot_poses, robot_goal, obstacle_poses, obs_orient, robot_vel=5, obstacle_vel = 5, curr_path = []):
        self.robot_poses = robot_poses
        self.robot_goal = robot_goal
        self.obstacle_poses = obstacle_poses
        self.obstacle_orient = obs_orient
        self.robot_vel = robot_vel
        self.obstacle_vel = obstacle_vel
        self.curr_path = curr_path

        self.num_sectors = 8 
        self.counts = [0 for i in range(self.num_sectors)]
        self.total_counts = [0 for i in range(self.num_sectors)]

        self.x_train = []
        self.y_train = []

    # def calculate_collision_time(self, obs_pos, rob_pos):
    #     agent_velocity = self.robot_vel
    #     obstacle_velocity = self.obstacle_vel

    #     if (agent_velocity > 0 and obstacle_velocity > 0) or (agent_velocity < 0 and obstacle_velocity < 0):
    #         return math.inf
    #     elif agent_velocity == 0 or obstacle_velocity == 0:
    #         return math.inf
    #     else:
    #         time_to_collision = abs(agent_velocity) / abs(obstacle_velocity)
    #         return time_to_collision
        
    def calculate_collision_time(self, obs_pos, rob_pos, obs_orientation, rob_orientation=None):
        # Convert orientation to radians if needed
        # obs_angle = math.radians(obs_orientation)
        # rob_angle = math.radians(rob_orientation)
        obs_angle = obs_orientation
        print(f'curr path: {self.curr_path} and obs angle: {obs_angle}')

        if rob_orientation == None: 
            rob_orientation = self.calc_goal_orientation(self.curr_path[0], self.curr_path[1])
        rob_angle = rob_orientation

        # Calculate the velocity components
        obs_velocity_x = self.obstacle_vel * math.cos(obs_angle)
        obs_velocity_y = self.obstacle_vel * math.sin(obs_angle)

        rob_velocity_x = self.robot_vel * math.cos(rob_angle)
        rob_velocity_y = self.robot_vel * math.sin(rob_angle)

        # Relative position and velocity
        relative_pos_x = obs_pos[0] - rob_pos[0]
        relative_pos_y = obs_pos[1] - rob_pos[1]

        relative_velocity_x = rob_velocity_x - obs_velocity_x
        relative_velocity_y = rob_velocity_y - obs_velocity_y

        # Calculate the parameters for the quadratic equation
        a = relative_velocity_x ** 2 + relative_velocity_y ** 2
        b = 2 * (relative_pos_x * relative_velocity_x + relative_pos_y * relative_velocity_y)
        c = relative_pos_x ** 2 + relative_pos_y ** 2

        # If both objects are moving in parallel, no collision will occur
        if a == 0:
            return math.inf

        # Calculate the discriminant
        discriminant = b ** 2 - 4 * a * c

        if discriminant < 0:
            return math.inf  # No collision

        # Calculate the times of collision (using only the positive root)
        t1 = (-b - math.sqrt(discriminant)) / (2 * a)
        t2 = (-b + math.sqrt(discriminant)) / (2 * a)

        # Gather valid times
        valid_times = [t for t in (t1, t2) if t >= 0]

        # Return the smallest positive time to collision, or infinity if no valid times
        return min(valid_times) if valid_times else math.inf


    def calc_goal_orientation(self, pos1, pos2): 
        x1, y1 = pos1
        x2, y2 = pos2
    
        # Calculate the differences
        delta_x = x2 - x1
        delta_y = y2 - y1
        
        # Calculate the angle using atan2
        theta = math.atan2(delta_y, delta_x)
        
        return theta 
    
    def update_counts(self):
        num_collide = 0 
        for i in range(len(self.obstacle_poses)):
            # determine if possible to collide 
            collide = 'no'
            collision_time = self.calculate_collision_time(self.obstacle_poses[i], self.robot_poses[i], self.obstacle_orient[i]) # using default val
            print(f'collision time: {abs(collision_time)}')
            if collision_time != math.inf: 
                ttg = self.time_to_goal(self.robot_poses[i], self.robot_goal)
                print(f'time to goal: {ttg}')
                print(ttg <= abs(collision_time))
                if ttg >= abs(collision_time): 
                    # id sector 
                    deg = math.degrees(self.obstacle_orient[i])
                    rang = self.sort_into_ranges(deg)
                    self.counts[rang] += 1
                    collide = 'yes'
                    num_collide += 1
                    print(f'yes: {collision_time}')

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

