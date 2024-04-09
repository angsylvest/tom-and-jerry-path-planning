import math
import heapq

from node import * 


# Define the grid parameters
GRID_SIZE = 100  # Size of the grid (e.g., 100x100)
STEP_SIZE = 1.0  # Step size for discretization
OBSTACLE_COST = float('inf')  # Cost for grid cells with obstacles
GOAL_TOLERANCE = 0.5  # Tolerance for reaching the goal


# Define the heuristic function (Euclidean distance)
def euclidean_distance(node, goal):

    if isinstance(node, Node):
        x1, y1 = node.position
    else: 
        x1, y1 = node
        
    x2, y2 = goal
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def generate_successors(env, pos):
    if not isinstance(pos, Node): 
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