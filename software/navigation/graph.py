from navigation.node import Node, StartNode, DepotNode, GoalNode, DummyNode


class Graph:
    def __init__(self):
        """
        Represents the navigation graph with weighted edges.
        """
        self.nodes = {}

    def add_node(self, name, node_type="dummy", action=None):
        """
        Adds a node to the graph with the specified type.
        """
        if name not in self.nodes:
            if node_type == "start":
                self.nodes[name] = StartNode(name, action=action)
            elif node_type == "depot":
                self.nodes[name] = DepotNode(name, action=action)
            elif node_type == "goal":
                self.nodes[name] = GoalNode(name, action=action)
            elif node_type == "dummy":
                self.nodes[name] = DummyNode(name, action=action)
            else:
                raise ValueError(f"Unknown node type: {node_type}")

    # A simple dictionary mapping each forward direction to its reverse
    

    def add_edge(self, from_node, to_node, cost, direction, junction=1):
        """
        Adds a weighted edge between two nodes, storing the 'direction' 
        for forward and 'reverse_direction' for the reverse edge.
        """
        REVERSE_DIRECTION = {
        1: 3,
        2: 4,
        3: 1,
        4: 2
        }
        
        if from_node not in self.nodes:
            self.add_node(from_node)
        if to_node not in self.nodes:
            self.add_node(to_node)

        from_node_obj = self.nodes[from_node]
        to_node_obj   = self.nodes[to_node]

        # Forward edge: store the original direction
        from_node_obj.add_neighbor(
            to_node_obj,
            cost,
            direction,
            junction
        )

        # Reverse edge: use the custom dictionary to get the reverse direction
        reverse_dir = REVERSE_DIRECTION[direction]
        to_node_obj.add_neighbor(
            from_node_obj,
            cost,
            reverse_dir,
            junction
        )


    def get_node(self, name):
        """Returns the node object for the given name."""
        return self.nodes.get(name, None)

    def __str__(self):
        """Returns a string representation of the graph."""
        return "\n".join([str(node) for node in self.nodes.values()])
    

def initialise_graph():
    # Initialise all nodes 
    full_graph = Graph()

    full_graph.add_node("Start Node",node_type="start")

    full_graph.add_node("Depot 1",node_type="depot")
    full_graph.add_node("Depot 2",node_type="depot")

    full_graph.add_node("A",node_type="goal")
    full_graph.add_node("B",node_type="goal")
    full_graph.add_node("C",node_type="goal")
    full_graph.add_node("D",node_type="goal")

    # Dummy intersection nodes
    full_graph.add_node("3",node_type="dummy")
    full_graph.add_node("4",node_type="dummy")
    full_graph.add_node("5",node_type="dummy")
    full_graph.add_node("6",node_type="dummy")
    full_graph.add_node("7",node_type="dummy")
    full_graph.add_node("8",node_type="dummy")

    # adding primary edges
    full_graph.add_edge("Start Node","Depot 1",105,direction=2)
    full_graph.add_edge("Start Node","A",32,direction=4)
    full_graph.add_edge("A","Depot 2",70,direction=4)
    full_graph.add_edge("Depot 2","3",85,direction=1)
    full_graph.add_edge("3","4",100,direction=2)
    full_graph.add_edge("4","B",34,direction=2)
    full_graph.add_edge("B","5",71,direction=2)
    full_graph.add_edge("5","8",78,direction=1)
    full_graph.add_edge("8","D",64,direction=4)
    full_graph.add_edge("D","7",42,direction=4)
    full_graph.add_edge("7","6",102,direction=4)
    full_graph.add_edge("Depot 1","5",85,direction=1)
    full_graph.add_edge("3","6",76,direction=1)
    full_graph.add_edge("4","C",37,direction=1)
    full_graph.add_edge("C","7",39,direction=1)

    # adding secondary edges

    # full_graph.add_edge("Start Node","Depot 2",102,direction=4,junction=2)
    # full_graph.add_edge("A","Depot 1",137,direction=2)
    # full_graph.add_edge("Depot 1","Depot 2",207,direction=4,junction=3)

    # full_graph.add_edge("Depot 2","6",161,direction=1,junction=2)
    # full_graph.add_edge("Depot 1","8",161,direction=1,junction=2)
    # full_graph.add_edge("4","7",76,direction=1,junction=2)

    # full_graph.add_edge("3","B",134,direction=2,junction=2)
    # full_graph.add_edge("4","5",105,direction=2,junction=2)
    # full_graph.add_edge("3","5",207,direction=2,junction=3)

    # full_graph.add_edge("6","8",207,direction=2,junction=3)
    # full_graph.add_edge("6","D",207,direction=2,junction=3)
    # full_graph.add_edge("7","8",106,direction=2,junction=2)
    
    return(full_graph)

if __name__ == "__main__":
    
    print(initialise_graph())
    
    
    

