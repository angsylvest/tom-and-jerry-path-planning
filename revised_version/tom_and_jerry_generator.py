import heapq
import math
from scipy.optimize import minimize_scalar
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

import sys 
sys.path.append('../../../revised_version')

from utility_func import * 
from grid_environment import * 
from multi_bayes import * 
from grid_environment import * 
from evolving_waypoint import * 
from risk_assessment import * 

class TomAndJerry: 

    def __init__(self, env, current_pos, goal_pos):
        self.class_index = 0 
        self.obstacle_velocity = 0.2
        self.size = 0.5
        self.distance_refuge = 0.7
        self.max_distance = 10 
        self.size = 0.5

        self.current_pos = current_pos
        self.goal_pos = goal_pos
        self.env = env 

        self.model = MultinomialNaiveBayes()

    def a_star_path(self, obstacle_pos = []):
        return dynamic_a_star(self.env, self.current_pos, self.goal_pos, obstacle_pos)
    
    def update_bayes_model(self, x_train, y_train, x_test):
        # Create an instance of MultinomialNaiveBayes
        #getting grasph of probability componenet
        model = MultinomialNaiveBayes()

        print(f'input x train: {x_train} and y train: {y_train}')

        # Train the model with training data
        model.fit(x_train, y_train)

        # Get class probabilities for test data
        class_probabilities = model.predict_proba(x_test)
        print("Class probabilities for test data:", class_probabilities)

        # Get predicted class for test data
        predicted_class = model.predict(x_test)
        print("Predicted class for test data:", predicted_class)

        specific_class_probability = class_probabilities[0][self.class_index]
        print(f"Probability for class {model.classes_[self.class_index]}:", specific_class_probability)

        #maximum amount of risk potential in e-puck robot (caluclations of speed potential, etc)
        max_risk = 2.3

        probability = specific_class_probability
        severity = (0.3304 * self.obstacle_velocity) + (0.2957 * self.size) + (0.3739 * self.distance_refuge)
        print("Severity Value:", severity)

        total_risk = probability * severity
        print("Total Risk:", total_risk)    

        #calculate c, B, and find optimized radius
        c = (max_risk - total_risk) / 10
        print("c value:", c)

        return predicted_class, specific_class_probability, severity, total_risk, c


    def reroute(self): # obstacle encountered
        pass 

    #splits current path postion in-half so that new positions can be appeneded into the list (waypoints and distance robots needs to keep away from obstacles)
    def list_splice(self, line1_points, intersection_point):
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
    

    def sort_into_8_ranges(self, radians_list):
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
    
    def proximity_to_goal(self, current_node, goal_node, max_distance_allowed):
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
    

    def is_isolated_or_clustered(self, nodes, current_node):
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
    def find_maximum_d(self, c, F, B, w, n, f = 0, m = 1):
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

    def calc_fid(self, line1_points, intersection_point, radians, prob, current_node, obst_pose, F=0.3, w=0.9, n=1, f=0, m=1):
        # TODO: explain this info and where it is coming from .. 

        #initalize dictionary
        sector_dict = {}
        # List of ranges as strings
        # keys = ['0-45', '45-90', '90-135', '135-180', '180-225', '225-270', '270-315', '315-360']

        #once detection is complete start risk analysis
        #split list in half and begin to append positions
        before_intersection, after_intersection = self.list_splice(line1_points, intersection_point)
        sorted_ranges = self.sort_into_8_ranges(radians) # TODO: what are sorted ranges? what are they here for? 
        
        print(sorted_ranges)
        #for each sector of the obstacle's movement
        # for sector in keys:
        for i in range(8):

            #maximum amount of risk potential in e-puck robot (caluclations of speed potential, etc)
            max_risk = 2.3

            probability = prob
            severity = (0.3304 * self.obstacle_velocity) + (0.2957 * self.size) + (0.3739 * self.distance_refuge)
            # print("Severity Value:", severity)

            total_risk = probability * severity
            # print("Total Risk:", total_risk)    

            #calculate c, B, and find optimized radius
            c = (max_risk - total_risk) / 10
            # print("c value:", c)

            # predicted_class, specific_class_probability, severity, total_risk, c = self.update_bayes_model(X_train, y_train, X_test)
            proximity_value = self.proximity_to_goal(self.current_pos, self.goal_pos, self.max_distance)

            result = self.is_isolated_or_clustered(line1_points, current_node) # TODO: what is this? 
            #b is equal to Benefit
            B = result[1] + proximity_value
            # print(B)

            optimized_radius = self.find_maximum_d(c, F, B, w, n, f, m)
            # print("Optimized Radius:", optimized_radius)

            #corresonding dictionary values for each
            sector_dict[f"Iteration_{i}"] = optimized_radius

        # print("Dictionary with optimized radius values:", sector_dict)

        distance = euclidean_distance(current_node, obst_pose)
        # print("Distance:", distance)
        return sector_dict, distance, after_intersection, optimized_radius
    


    def distance(self, point1, point2):
        print(type(point2))
        x1, y1 = point1 
        x2, y2 = point2
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def find_tangent_point(self, center, radius, point):
        # Calculate the direction from the center to the point
        direction = np.array(point) - np.array(center)
        direction = direction / np.linalg.norm(direction)  # Normalize
        # Calculate the tangent point
        tangent_point = np.array(center) + direction * radius
        return tangent_point

    def better_generate_waypoints(self, original_path, obstacle_position, radius):
        new_path = []
        obstacle_radius = radius
        obstacle_center = obstacle_position# np.array(obstacle_position)
        curve_points = []

        print(f'original path: {original_path}')

        for i in range(len(original_path) - 1):
            start = original_path[i] # np.array(original_path[i])
            end = original_path[i+1] # np.array(original_path[i + 1])

            print(type(start), type(end), (obstacle_center))

            # Check distance from both start and end to the obstacle
            if self.distance(start, obstacle_center) < obstacle_radius or self.distance(end, obstacle_center) < obstacle_radius:
                # If either point is inside the obstacle's clearance zone
                tangent_start = self.find_tangent_point(obstacle_center, obstacle_radius, start)
                tangent_end = self.find_tangent_point(obstacle_center, obstacle_radius, end)

                # Calculate points to navigate around the obstacle
                curve_center = obstacle_center
                curve_radius = obstacle_radius
                
                # Append the start point to the new path
                new_path.append(start)

                # Generate curve points (simplified circular arc for this example)
                curve_angle_start = np.arctan2(tangent_start[1] - curve_center[1], tangent_start[0] - curve_center[0])
                curve_angle_end = np.arctan2(tangent_end[1] - curve_center[1], tangent_end[0] - curve_center[0])
                num_curve_points = 10
                curve_angles = np.linspace(curve_angle_start, curve_angle_end, num_curve_points)

                for angle in curve_angles:
                    curve_point = curve_center + np.array([curve_radius * np.cos(angle), curve_radius * np.sin(angle)])
                    new_path.append(curve_point)
                    curve_points.append(curve_point)

                # Append the end tangent point to the new path
                new_path.append(tangent_end)
            else:
                # No need to modify the path segment
                new_path.append(start)

        # Append the last point in the original path
        new_path.append(original_path[-1])

        return np.array(new_path), np.array(curve_points)
            
        
    def generate_waypoints(self, obs_pose, dist, start_pos, goal_pos ): 
        tangent_start, tangent_end, marked_coordinates = get_circle_paths_and_coordinates(obs_pose, dist, start_pos, goal_pos) # FID radius, center is where obstacle is 
        return tangent_start, tangent_end, marked_coordinates


# need steps that were taken to generate path .. 

def main():
    # generating example path here .. 
    start_pos = (0,0)
    goal_pos = (0.5,0.5)

    env = TomAndJerryEnvironment(dims = (2,2), upper_left=(-1,1),grid_dim=0.2) 
    # print(f'discretized env: {env}')

    # begin by generating a* path using fully connected graph 
    path_generator = TomAndJerry(env=env, current_pos=start_pos, goal_pos=goal_pos)
    curr_path = path_generator.a_star_path()
    print(f'a* path: {curr_path}')

    rob_poses = [(0,0), (0,0), (0,0), (0,0), (0,0), (0,0)]
    obst_poses =[(1,0), (1,0), (1,0), (1,0), (0.5,0.5), (0,0)]
    obs_orient = [0.785398, 1.5708, 2.35619, 3.14159, 3.92699, 3.92699]
    risk_assessment = ObstacleAssessment(robot_poses=rob_poses, robot_goal=goal_pos, obstacle_poses=obst_poses, obs_orient=obs_orient, curr_path = curr_path)
    # calc prob of collision given info about robot goals, and obstacle behavior (limited view)
    x_train, y_train, prob = risk_assessment.update_counts()
    # print(f'updated counts given obs info: {risk_assessment.update_counts()}')

    # calculate probability using given info 


    # # --- calculate fid if encounter obs ----- 
    # # TODO: what is this info? 
    # radians = [0.785398, 1.5708, 2.35619, 3.14159, 3.92699, 4.71239, 5.49779, 6.28319]
    # X_train = np.array([[0, 3, 4, 0, 2],
    #                 [0, 1, 4, 0, 2],
    #                 [0, 3, 4, 0, 2],
    #                 [0, 3, 4, 0, 2],
    #                 [0, 3, 4, 0, 2]])
    # y_train = np.array(['yes', 'no', 'yes', 'yes', 'no'])
    # X_test = np.array([[0, 3, 4, 0, 2]])

    obs_pose = (3,3)
    sector, dist, other, optim_rad = path_generator.calc_fid(curr_path, obst_poses[-1], obs_orient, prob, start_pos, obs_pose) # inputs: current path, 
    print(f'calc fid output: {optim_rad}')

    # # create path based on location of obstacle and other info ..
    # # TODO: where is this info? 
    radius = optim_rad # 0.35 # radius of e-puck robot

    # will re-do circular path, bc this doesn't make any sense .. 
    # tangent_start, tangent_end, marked_coordinates = get_circle_paths_and_coordinates(obs_pose, radius, start_pos, goal_pos) # FID radius, center is where obstacle is 
    # print(f'evolving waypoint generated around obstacle: {tangent_start, tangent_end, marked_coordinates}')

    print(f'currpath: {curr_path}, and others {obs_pose[-1], optim_rad}')
    marked_coordinates, curve_points = path_generator.better_generate_waypoints(curr_path, obst_poses[-1], optim_rad)
    print(f'curve points: {curve_points}')
    render_results(curr_path, obst_poses[-1], marked_coordinates, curve_points)


def render_results(rob_path, intersec, waypoints, curve_points):
    # Unpack robot path coordinates
    rob_x, rob_y = zip(*rob_path)
    
    # Unpack intersection point coordinates
    intersec_x, intersec_y = intersec
    
    # Unpack waypoint coordinates
    waypoints_x, waypoints_y = zip(*waypoints)

    # unpack circular points
    circ_x, circ_y = zip(*curve_points)

    # Create the plot
    plt.figure(figsize=(10, 6))

    # Plot the robot path
    plt.plot(rob_x, rob_y, label='Robot Path', color='blue', linewidth=2)

    # Plot the intersection point
    plt.scatter(intersec_x, intersec_y, color='red', s=100, label='Intersection Point', edgecolor='black')

    # Plot the waypoints
    plt.scatter(waypoints_x, waypoints_y, color='green', label='Waypoints', s=50)

    # Plot the waypoints
    plt.scatter(circ_x, circ_y, color='blue', label='circl', s=50)

    # Add labels and title
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Robot Path, Intersection, and Waypoints')
    plt.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    plt.axvline(0, color='gray', linewidth=0.5, linestyle='--')
    plt.grid()
    plt.legend()
    plt.axis('equal')  # Keep the aspect ratio square

    # Show the plot
    plt.show()

# main()
