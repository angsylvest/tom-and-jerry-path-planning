from node import * 

class TomAndJerryEnvironment:

    def __init__(self, dims=(), upper_left=(), grid_dim = 0.2):
        self.grid_dim = grid_dim
        self.dims = dims
        self.upper_left = upper_left
        self.nodes = self.create_nodes()
        # print(self.nodes)

    def create_nodes(self):
        nodes = []
        for i in range(int(self.dims[0] / self.grid_dim)):
            for j in range(int(self.dims[1] / self.grid_dim)):
                x = round(self.upper_left[0] + (i * self.grid_dim),2)
                y = round(self.upper_left[1] - (j * self.grid_dim),2)
                nodes.append(Node((x, y), []))  # Assuming neighbors are empty initially
        self.set_node_neighbors(nodes)
        return nodes

    def set_node_neighbors(self, nodes):
        for node in nodes:
            node_x, node_y = node.position
            for other_node in nodes:
                if other_node != node:
                    other_x, other_y = other_node.position
                    if (abs(node_x - other_x) <= self.grid_dim + 0.001) and \
                        (abs(node_y - other_y) <= self.grid_dim + 0.001):
                        node.neighbors.append(other_node)


    def get_node_at_position(self, pos): 
        for node in self.nodes:
            node_x, node_y = node.position
            if node_x <= pos[0] <= node_x + self.grid_dim and node_y <= pos[1] <= node_y + self.grid_dim:
                return node
        return None  # Position not within any node