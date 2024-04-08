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

import sys 
sys.path.append('../../../')

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
    
# Define the grid parameters
GRID_SIZE = 100  # Size of the grid (e.g., 100x100)
STEP_SIZE = 1.0  # Step size for discretization
OBSTACLE_COST = float('inf')  # Cost for grid cells with obstacles
GOAL_TOLERANCE = 0.5  # Tolerance for reaching the goal

# Define the heuristic function (Euclidean distance)
def euclidean_distance(node, goal):
    x1, y1 = node
    x2, y2 = goal
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

# Define a function to generate successors for a given node
def generate_successors(node):
    x, y = node
    successors = []

    # Define the possible movements (up, down, left, right, and diagonals)
    moves = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]

    for dx, dy in moves:
        new_x, new_y = x + dx * STEP_SIZE, y + dy * STEP_SIZE

        # Check if the new position is within the grid boundaries
        if 0 <= new_x < GRID_SIZE and 0 <= new_y < GRID_SIZE:
            successors.append((new_x, new_y))

    return successors

# Dynamic A* algorithm
def dynamic_a_star(start, goal, obstacles):
    open_set = [(0, start)]  # Priority queue with (f, node)
    came_from = {}  # Parent nodes (determines where previous node came from)
    g_score = {node: float('inf') for node in obstacles}  # Cost from start to node
    g_score[start] = 0
    g_score[goal] = float('inf')  # Initialize goal with an infinite cost
    # g_score is a dictionary which keeps track of the cost of movement from starting node to each explored node (which is the key/value pair)
    # for example g_score[(1,1)] might correspond to the euclidean_distance value 

    while open_set:
        _, current = heapq.heappop(open_set) # remove the node with the lowest f-score 

        # if we found a path to reach the goal, we will backtrack to find the path created 
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for neighbor in generate_successors(current):
            if neighbor in obstacles:
                continue

            tentative_g_score = g_score[current] + euclidean_distance(current, neighbor)

            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + euclidean_distance(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return None  # No path found
    
   

start = (0, 0)
goal = (10, 10)
#static obstalces
obstacles = [(2, 2), (3, 3), (4, 4), (5, 5)]  # Example obstacle positions

class MultinomialNaiveBayes:
    def __init__(self, alpha=1):
        self.alpha = alpha  # Laplace smoothing parameter
        self.class_prior_ = None
        self.feature_prob_ = None

    def fit(self, X, y):
        self.classes_ = np.unique(y)
        n_classes = len(self.classes_)
        n_features = X.shape[1]

        # Calculate class prior probabilities
        self.class_prior_ = np.zeros(n_classes)
        for i, c in enumerate(self.classes_):
            self.class_prior_[i] = np.sum(y == c) / float(len(y))

        # Calculate smoothed feature probabilities
        self.feature_prob_ = np.zeros((n_classes, n_features))
        for i, c in enumerate(self.classes_):
            class_instances = X[y == c]
            total_count = np.sum(class_instances) + n_features * self.alpha
            self.feature_prob_[i] = (np.sum(class_instances, axis=0) + self.alpha) / total_count

    def predict_proba(self, X):
        return np.exp(self.predict_log_proba(X))

    def predict_log_proba(self, X):
        n_classes = len(self.classes_)
        n_samples, n_features = X.shape
        log_prob = np.zeros((n_samples, n_classes))

        for i in range(n_samples):
            for j in range(n_classes):
                log_prob[i, j] = np.sum(np.log(self.feature_prob_[j]) * X[i])

        log_prob += np.log(self.class_prior_)
        return log_prob


    def predict(self, X):
        return self.classes_[np.argmax(self.predict_log_proba(X), axis=1)]
    
#splits current path postion in-half so that new positions can be appeneded into the list (waypoints and distance robots needs to keep away from obstacles)
def list_splice(line1_points, intersection_point):
    # Convert intersection_point to a tuple if it's an integer
    if isinstance(intersection_point, int):
        intersection_point = (intersection_point,)

    # Check if the intersection point is in Line 1 points and remove it
    if intersection_point in line1_points:
        line1_points.remove(intersection_point)

    # Initialize variables to store the spliced lists
    before_intersection_line1 = []
    after_intersection_line1 = []

    # Flag to track whether the intersection has been processed
    intersection_processed = False

    # Check between which points the intersection lies on Line 1
    for i in range(len(line1_points) - 1):
        (x1_start, y1_start) = line1_points[i]
        (x1_end, y1_end) = line1_points[i + 1]

        # Convert integers to tuples for comparison
        if not isinstance(intersection_point, tuple):
            intersection_point = (intersection_point,)

        # Check if the intersection point lies within the bounding box of the two consecutive points
        if (
            min(x1_start, x1_end) <= intersection_point[0] <= max(x1_start, x1_end) and
            min(y1_start, y1_end) <= intersection_point[1] <= max(y1_start, y1_end)
        ):
            # Check if the intersection has already been processed
            if not intersection_processed:
                print(f"Intersection is between points ({x1_start}, {y1_start}) and ({x1_end}, {y1_end}) on Line 1.")
                intersection_processed = True  # Set the flag to True

            # Splice the list between the two points
            index = line1_points.index((x1_start, y1_start))
            before_intersection_line1 = line1_points[: index + 1]
            after_intersection_line1 = line1_points[index + 1:]

            # Exclude the intersection point from both lists
            before_intersection_line1 = [point for point in before_intersection_line1 if point != intersection_point]
            after_intersection_line1 = [point for point in after_intersection_line1 if point != intersection_point]

    # Return both spliced lists
    return before_intersection_line1, after_intersection_line1

#sort raidan movement into 8 equidiscent sectors

def sort_into_8_ranges(radians_list):
    # Convert radians to degrees
    degrees_list = [math.degrees(rad) for rad in radians_list]

    # Sort the list of degrees
    degrees_list.sort()

    # Initialize the 8 ranges
    ranges = [[] for _ in range(8)]

    # Iterate through the sorted degrees and assign them to the appropriate range
    for degree in degrees_list:
        sector = int(degree // 45)
        if sector == 8:  # Wrap around if degree is 360
            sector = 0
        ranges[sector].append(math.radians(degree))

    return ranges

#Benefit Component
#component 1

#proximity to goal

def proximity_to_goal(current_node, goal_node, max_distance_allowed):
    """
    Calculate the proximity of a current node to the goal node using an exponential function.
    
    Args:
    - current_node: The current node position.
    - goal_node: The goal node position.
    - max_distance_allowed: The maximum distance allowed.
    
    Returns:
    - Proximity value between 0 and 1, where 1 indicates the current node is at the goal, 
      and values closer to 1 indicate decreasing distance from the goal.
    """
    distance = math.sqrt((current_node[0] - goal_node[0])**2 + (current_node[1] - goal_node[1])**2)
    proximity = 1 - math.exp(-distance * (1 / max_distance_allowed))
    return proximity

# Example usage:
current_position = (3, 4)
goal_position = (7, 8)
max_distance_allowed = 10  # Decrease this value linearly

proximity_value = proximity_to_goal(current_position, goal_position, max_distance_allowed)
print("Proximity to goal:", proximity_value)

#component 2 

#isolated or clustered

import numpy as np

def is_isolated_or_clustered(nodes, current_node):
    # Get the neighboring nodes of the current node
    neighbors = [node for node in nodes if node != current_node]

    if len(neighbors) == 0:
        return "Isolated", std_dev  # Current node has no neighbors

    # Calculate distances between the current node and its neighboring nodes
    distances = [np.linalg.norm(np.array(current_node) - np.array(neighbor)) for neighbor in neighbors]

    # Calculate the standard deviation of distances
    std_dev = np.std(distances)

    # Determine if the current node is isolated or clustered based on standard deviation
    if std_dev < 0.5:  # You can adjust this threshold as needed
        return "Clustered", std_dev
    else:
        return "Isolated", std_dev
    
#Flight Initiation Distance Equation
def find_maximum_d(c, F, B, w, n, f = 0, m = 1):
    # Define the model and its derivative
    def model(d):
        return (F + B * ((1 - d / w) ** n) - f * (d ** m)) * (1 - np.exp(-c * d))

    def model_derivative(d):
        return (c * np.exp(-c * d) * (F + B * (1 - (d / w) ** n) - f * d ** m)) + ((1 - np.exp(-c * d)) * (n * B * (d ** (n - 1)) / (w ** n) - m * f * (d ** (m - 1))))

    # Find the critical point (where the derivative is zero)
    result_critical = minimize_scalar(model_derivative, bounds=(0, w), method='bounded')
    d_critical = result_critical.x

    # Optionally, you can also use optimization to find the maximum directly
    result_max = minimize_scalar(lambda d: -model(d), bounds=(0, w), method='bounded')
    d_max = result_max.x

    return d_max

#calculate distance between obstacle postion and current poistion (current_node)
def euclidean_distance(coord1, coord2):
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    return round(sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2), 2)

def main( detection, line1_points, intersection_point, radians, X_train, y_train, X_test, obstacle_velocity, size, distance_refuge, nodes, current_node, F, w, n, f, m, coord1, coord2):

#initalize dictionary
    sector_dict = {}
# List of ranges as strings
# keys = ['0-45', '45-90', '90-135', '135-180', '180-225', '225-270', '270-315', '315-360']



    #once detection is complete start risk analysis
    #split list in half and begin to append positions
    before_intersection, after_intersection = list_splice(line1_points, intersection_point)
    sorted_ranges = sort_into_8_ranges(radians)
    print(sorted_ranges)
    #for each sector of the obstacle's movement
    # for sector in keys:
    for i in range(8):
            # Create an instance of MultinomialNaiveBayes
            #getting grasph of probability componenet
        model = MultinomialNaiveBayes()

        # Train the model with training data
        model.fit(X_train, y_train)

        # Get class probabilities for test data
        class_probabilities = model.predict_proba(X_test)
        print("Class probabilities for test data:", class_probabilities)

        # Get predicted class for test data
        predicted_class = model.predict(X_test)
        print("Predicted class for test data:", predicted_class)

        specific_class_probability = class_probabilities[0][class_index]
        print(f"Probability for class {model.classes_[class_index]}:", specific_class_probability)

        #maximum amount of risk potential in e-puck robot (caluclations of speed potential, etc)
        max_risk = 2.3

        probability = specific_class_probability
        severity = (0.3304 * obstacle_velocity) + (0.2957 * size) + (0.3739 * distance_refuge)
        print("Severity Value:", severity)

        total_risk = probability * severity
        print("Total Risk:", total_risk)    

        #calculate c, B, and find optimized radius

        c = (max_risk - total_risk) / 10
        print("c value:", c)

        result = is_isolated_or_clustered(nodes, current_node)
        #b is equal to Benefit
        B = result[1] + proximity_value
        print(B)

        optimized_radius = find_maximum_d(c, F, B, w, n, f, m)
        print("Optimized Radius:", optimized_radius)

        #corresonding dictionary values for each
        sector_dict[f"Iteration_{i}"] = optimized_radius

    print("Dictionary with optimized radius values:", sector_dict)

    distance = euclidean_distance(coord1, coord2)
    print("Distance:", distance)
    return sector_dict, distance
    
if __name__ == "__main__":
    # Example usage
    # Example usage
    
    detection = 1
    line1_points = __path__
    #intersection point is the point at which the robot will begin to diverege from its original route
    intersection_point = (0,0)
    #are appended to the robot's knowledge as it watches the obstacle move back and fourth from one position to another
    radians = [0.785398, 1.5708, 2.35619, 3.14159, 3.92699, 4.71239, 5.49779, 6.28319]  # Some example radians
    #represents number of times the obstacle travels in each sectored "range"
    #x train should technically be corresponding to the secotor raidans
    #premature values to make data more robust
    X_train = np.array([[0, 3, 4, 0, 2],
                        [0, 1, 4, 0, 2],
                        [0, 3, 4, 0, 2],
                        [0, 3, 4, 0, 2],
                        [0, 3, 4, 0, 2]])
    y_train = np.array(['yes', 'no', 'yes', 'yes', 'no'])
    X_test = np.array([[0, 3, 4, 0, 2]])
    class_index = 0
    obstacle_velocity = 0.2
    size = 0.5
    distance_refuge = 0.7
    nodes = __path__
    #robot.get position
    current_node = (3,3)
    #robot's maximal speed capacity
    F = 0.3
    #detection distance
    w = 0.9
    #enviornmental urgency component
    n = 1
    f = 0
    m = 1
    #obstacle's current position
    #continuously updated
    coord1 = (3,4)
    #robot's current poisiton
    coord2 = current_node
    
    obj_detected = True

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
i = 0 
while robot.step(timestep) != -1:

    path = dynamic_a_star(start, goal, obstacles)
    if path:
        print("Path found:", path)
        if obj_detected:
            main( detection, line1_points, intersection_point, radians, X_train, y_train, X_test, obstacle_velocity, size, distance_refuge, nodes, current_node, F, w, n, f, m, coord1, coord2)
            if distance < sector_dict:
                move_forward()

        
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
        # print(chosen_direction)
                
        if yaw != chosen_direction and not goal_reached: 
            begin_rotating()
                    
        elif yaw == chosen_direction and not goal_reached: 
            move_forward() 
            
    else:
        print("No path found.")


   
    
    i += 1
    pass

# Enter here exit cleanup code.
