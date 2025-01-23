from navigation.node import Node

class Graph:
    def __init__(self):
        """
        Represents the navigation graph with weighted edges.
        """
        self.nodes = {}

    def add_node(self, name, node_type="dummy", action=None):
        """
        Adds a node to the graph.

        Args:
            name (str): Name or ID of the node.
            node_type (str): Type of node ("start", "depot", "goal", "dummy").
            action (callable): Function to execute when at this node.
        """
        if name not in self.nodes:
            self.nodes[name] = Node(name, node_type, action=action)

    def add_edge(self, from_node, to_node, cost=1):
        """
        Adds a weighted edge between two nodes.

        Args:
            from_node (str): Name of the starting node.
            to_node (str): Name of the ending node.
            cost (int): Weight or cost of the edge.
        """
        if from_node not in self.nodes:
            self.add_node(from_node)
        if to_node not in self.nodes:
            self.add_node(to_node)

        self.nodes[from_node].add_neighbor(self.nodes[to_node], cost)
        self.nodes[to_node].add_neighbor(self.nodes[from_node], cost)  # Bidirectional edge

    def get_node(self, name):
        """Returns the node object for the given name."""
        return self.nodes.get(name, None)

    def __str__(self):
        """Returns a string representation of the graph."""
        return "\n".join([str(node) for node in self.nodes.values()])
