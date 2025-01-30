class Node:
    def __init__(self, name, node_type="dummy", neighbors=None, action=None):
        """
        Represents a single node in the graph.

        Args:
            name (str): Unique name or identifier for the node (e.g., "Start", "Depot1").
            node_type (str): Type of node ("S", "D", "G", or "J").
            neighbors (dict): Dictionary of neighboring nodes and their costs (e.g., {"B": 1, "C": 2}).
            action (callable): A function or method to execute when the robot reaches this node.
        """
        self.name = name
        self.node_type = node_type  # "start", "depot", "goal", or "dummy"
        self.neighbors = neighbors or {}  # Neighboring nodes with edge weights
        self.action = action  # Function to execute when at this node

    def __str__(self):
        # Collect just the neighbor names (or IDs)
        neighbor_names = [neighbor.name for neighbor in self.neighbors.keys()]
        return f"Node({self.name}, Type: {self.node_type}, Neighbors: {neighbor_names})"

    def add_neighbor(self, neighbor, cost, direction, junction):
        self.neighbors[neighbor] = {
            "cost": cost,
            "direction": direction,
            "junction": junction
        }
    def execute_action(self):
        """Executes the action associated with this node."""
        if self.action:
            self.action()
        else:
            print(f"No action defined for node: {self.name}")

    def __str__(self):

        neighbor_names = [nbr.name for nbr in self.neighbors.keys()]
        return f"Node({self.name}, Type: {self.node_type}, Neighbors: {neighbor_names})"

    def __repr__(self):
     
        return f"Node({self.name})"
    
# Creat subclasses of nodes
class StartNode(Node):
    def __init__(self, name, neighbors=None, action=None):
        super().__init__(name, node_type="start", neighbors=neighbors, action=action)
        self.node_type = "start"

    def execute_action(self):
        # Insert action 
        super().execute_action()


class DepotNode(Node):
    def __init__(self, name, neighbors=None, action=None):
        super().__init__(name, node_type="depot", neighbors=neighbors, action=action)
        self.node_type = "depot"

    def execute_action(self):
        # Insert action 
        print("This is the start node")
        super().execute_action()
        

class GoalNode(Node):
    def __init__(self, name, neighbors=None, action=None):
        super().__init__(name, node_type="goal", neighbors=neighbors, action=action)
        self.node_type = "goal"

    def execute_action(self):
        # Insert action 
        super().execute_action()


class DummyNode(Node):
    def __init__(self, name, neighbors=None, action=None):
        super().__init__(name, node_type="dummy", neighbors=neighbors, action=action)
        self.node_type = "dummy"

    def execute_action(self):
       # Insert action 
        super().execute_action()