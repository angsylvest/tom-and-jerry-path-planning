import math
import heapq

import sys 
sys.path.append('../')
from revised_version.node import * 


# Define the grid parameters
GRID_SIZE = 100  # Size of the grid (e.g., 100x100)
STEP_SIZE = 1.0  # Step size for discretization
OBSTACLE_COST = float('inf')  # Cost for grid cells with obstacles
GOAL_TOLERANCE = 0.5  # Tolerance for reaching the goal

# translation function used for D* lite .. 
def real_to_grid_pos(real_pos=(0,0), env_size=(1,1), upper_left_corner=(-0.5, 0.5), grid_size = 0.2):
    # Calculate the grid dimensions
    x_dim = int(env_size[0] / grid_size)
    y_dim = int(env_size[1] / grid_size)
    
    # Calculate the position offset based on the upper left corner
    offset_x = upper_left_corner[0]
    offset_y = upper_left_corner[1]
    
    # Calculate the grid position
    grid_x = int((real_pos[0] - offset_x) / grid_size)
    grid_y = int((offset_y - real_pos[1]) / grid_size)  # y-axis is inverted in the grid
    
    # Ensure the grid position is within bounds
    grid_x = max(0, min(x_dim - 1, grid_x))
    grid_y = max(0, min(y_dim - 1, grid_y))
    
    return grid_x, grid_y

def grid_to_real_pos(grid_pos=(0,0), env_size=(1,1), upper_left_corner=(-0.5, 0.5), grid_size=0.2):
    # Calculate the grid dimensions
    x_dim = int(env_size[0] / grid_size)
    y_dim = int(env_size[1] / grid_size)
    
    # Calculate the position offset based on the upper left corner
    offset_x = upper_left_corner[0]
    offset_y = upper_left_corner[1]
    
    # Calculate the real x and y coordinates
    real_x = grid_pos[0] * grid_size + offset_x
    real_y = offset_y - grid_pos[1] * grid_size  # y-axis is inverted in the grid
    
    # Ensure the real coordinates are within bounds
    real_x = max(offset_x, min(env_size[0] + offset_x, real_x))
    real_y = max(offset_y - env_size[1], min(offset_y, real_y))
    
    return real_x, real_y

# Define the heuristic function (Euclidean distance)
def euclidean_distance(node, goal):

    if isinstance(node, Node):
        x1, y1 = node.position
    else: 
        x1, y1 = node
        
    x2, y2 = goal
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def generate_successors(env, pos):
    # print(f'in generate sucessors: {pos} for type {type(pos)}')
    if not isinstance(pos, Node) and isinstance(pos, tuple): 
        # print(f'is not a ndoe')
        node = env.get_node_at_position(pos)
    else: 
        node = pos 

    return node.neighbors

def dynamic_a_star(env, start_pos, goal_pos, obstacle_pos):

    start_node = env.get_node_at_position(start_pos)
    goal_node = env.get_node_at_position(goal_pos)
    
    obstacle_nodes = [env.get_node_at_position(pos) for pos in obstacle_pos]

    open_set = [(0, start_node)]  # Priority queue with (f, node)
    came_from = {}  # Parent nodes (determines where previous node came from)
    g_score = {node: float('inf') for node in obstacle_nodes}  # Cost from start to node
    g_score[start_node] = 0
    g_score[goal_node] = float('inf')  # Initialize goal with an infinite cost

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal_node:
            path = []
            while current in came_from:
                path.append(current.position)  # Append the position associated with the node
                current = came_from[current]
            path.append(start_node.position)
            path.reverse()
            return path

        for neighbor in generate_successors(env, current):
            # print(f'neighbor {neighbor}')
            if neighbor in obstacle_nodes:
                continue

            tentative_g_score = g_score[current] + euclidean_distance(current.position, neighbor.position)

            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                # print(f'neighbor: {neighbor} and goal node {goal_node}')
                f_score = tentative_g_score + euclidean_distance(neighbor.position, goal_node.position)
                heapq.heappush(open_set, (f_score, neighbor))

    return None  # No path found


# relevant metrics to add here: path taken, time to goal, etc