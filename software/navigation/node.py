class Node:
    def __init__(self, name, node_type="dummy", neighbors=None, action=None):
        """
        Represents a single node in the graph.

        Args:
            name (str): Unique name or identifier for the node (e.g., "Start", "Depot1").
            node_type (str): Type of node ("start", "depot", "goal", or "dummy").
            neighbors (dict): Dictionary of neighboring nodes and their costs (e.g., {"B": 1, "C": 2}).
            action (callable): A function or method to execute when the robot reaches this node.
        """
        self.name = name
        self.node_type = node_type  # "start", "depot", "goal", or "dummy"
        self.neighbors = neighbors or {}  # Neighboring nodes with edge weights
        self.action = action  # Function to execute when at this node

    def add_neighbor(self, neighbor, cost=1):
        """Adds a neighboring node with a specified cost."""
        self.neighbors[neighbor] = cost

    def execute_action(self):
        """Executes the action associated with this node."""
        if self.action:
            self.action()
        else:
            print(f"No action defined for node: {self.name}")

    def __str__(self):
        return f"Node({self.name}, Type: {self.node_type}, Neighbors: {list(self.neighbors.keys())})"

    def __repr__(self):
        return self.__str__()