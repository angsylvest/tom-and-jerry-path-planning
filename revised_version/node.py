class Node:
    def __init__(self, pos, neigh=[]):
        self.position = pos
        self.neighbors = neigh
        self.is_occupied_by_obstacle = False

    def set_neighbors(self, nodes):
        self.neighbors = []
        for other_node in nodes:
            if other_node != self:
                distance = ((self.position[0] - other_node.position[0]) ** 2 +
                            (self.position[1] - other_node.position[1]) ** 2) ** 0.5
                if distance <= 0.3:  # Adjust this threshold as needed for neighbor connectivity
                    self.neighbors.append(other_node)

    def get_neighbors(self):
        return self.neighbors
    
    def __str__(self):
        return str(self.position)
    
    def __lt__(self, other):
        # Define how nodes should be compared based on their positions
        return self.position < other.position
    
    def update_occupation(self, occupied):
        self.is_occupied_by_obstacle = occupied