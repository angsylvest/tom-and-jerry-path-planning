import heapq

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