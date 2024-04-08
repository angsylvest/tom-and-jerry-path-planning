"""d_star_lite controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import heapq

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

class LifelongAStar:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.inf = float('inf')

    def heuristic(self, current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def get_neighbors(self, node):
        neighbors = []
        for i, j in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            x, y = node[0] + i, node[1] + j
            if 0 <= x < self.rows and 0 <= y < self.cols and self.grid[x][y] != 1:
                neighbors.append((x, y))
        return neighbors

    def update_obstacle(self, obstacle_position):
        x, y = obstacle_position
        if 0 <= x < self.rows and 0 <= y < self.cols:
            self.grid[x][y] = 1

    def lifelong_astar(self, start, goal):
        open_set = [(0, start)]
        g_values = {start: (0, None)}

        while open_set:
            current_cost, current_node = heapq.heappop(open_set)

            if current_node == goal:
                path = []
                while current_node in g_values:
                    path.insert(0, current_node)
                    current_node = g_values[current_node][1]
                return path

            for neighbor in self.get_neighbors(current_node):
                new_g = g_values[current_node][0] + self.grid[neighbor[0]][neighbor[1]]

                if neighbor not in g_values or new_g < g_values[neighbor][0]:
                    g_values[neighbor] = new_g, current_node
                    heapq.heappush(open_set, (new_g + self.heuristic(neighbor, goal), neighbor))

        return None
        
grid = [
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1],
    [0, 0, 0, 0, 1],
    [0, 0, 0, 0, 1],
    [0, 1, 0, 0, 0],
]



path = lifelong_astar.lifelong_astar(start_node, goal_node)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    
    lifelong_astar = LifelongAStar(grid)
    start_node = (0, 4)
    goal_node = (4, 4)

    path = lifelong_astar.lifelong_astar(start_node, goal_node)

    if path:
        print("Path found:", path)
    else:
        print("No path found.")

# Example of updating obstacle position
    new_obstacle_position = (2, 1)  # Example new obstacle position
    lifelong_astar.update_obstacle(new_obstacle_position)

# Recalculate path after updating obstacle
    new_path = lifelong_astar.lifelong_astar(start_node, goal_node)
    if new_path:
        print("New path found:", new_path)
    else:
        print("No path found after obstacle update.")
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
