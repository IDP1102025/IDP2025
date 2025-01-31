from graph import Graph, initialise_graph
import heapq
from collections import deque
class Navigation:
    def __init__(self):
        self.graph = initialise_graph()

    def dijkstra_with_directions(self, start_name, goal_name):
        """Return (distance, node_path, dir_path)."""
        
        start_node = self.graph.get_node(start_name)
        goal_node  = self.graph.get_node(goal_name)
        
        # 1) Distances dict from Node obj -> cost
        distances = {node_obj: float('inf') for node_obj in self.graph.nodes.values()}
        distances[start_node] = 0

        # 2) came_from dict from Node obj -> (prev_node_obj, direction)
        came_from = {node_obj: None for node_obj in self.graph.nodes.values()}

        # 3) Priority queue: Store (distance, node.name, node_obj) to allow sorting
        pq = [(0, start_node.name, start_node)]  # (distance_so_far, tie-breaker, node_obj)

        while pq:
            current_dist, _, current_node = heapq.heappop(pq)
            
            # If we've reached the goal node, we can stop
            if current_node == goal_node:
                break

            if current_dist > distances[current_node]:
                continue
            
            # 4) Explore neighbors
            for neighbor_obj, edge_data in current_node.neighbors.items():
                cost = edge_data["cost"]
                direction = edge_data["direction"]

                new_dist = current_dist + cost
                if new_dist < distances[neighbor_obj]:
                    distances[neighbor_obj] = new_dist
                    came_from[neighbor_obj] = (current_node, direction)
                    heapq.heappush(pq, (new_dist, neighbor_obj.name, neighbor_obj))  # Store tuple with tie-breaker

        # 5) Reconstruct path of Node objects + directions
        node_path = []
        dir_path = []

        node = goal_node
        while node is not None:
            node_path.append(node)
            prev_info = came_from[node]
            if prev_info is not None:
                prev_node, used_direction = prev_info
                dir_path.append((used_direction, 1))
                node = prev_node
            else:
                node = None
        
        # Reverse them, since we built from goal -> start
        node_path.reverse()
        dir_path.reverse()

        # Combine adjacent nodes that have the same direction of travel
        node_path, dir_path = combine_paths(node_path, dir_path)

        return (distances[goal_node], node_path, dir_path)


    def find_shortest_path_with_hardcode(self,start_node_name,end_node_name):
        '''
        Args:
            start_node_name (str): Name of the starting node.
            end_node_name (str): Name of the ending node.

        Returns:
            direction_list: List of tuples containing each (direction to travel and junctions to pass)
            node_list: List of nodes to go from start to end

        List of paths:


        '''

        node_path_dict = {
            ('Start', )
        }

        raise NotImplementedError
    

def combine_paths(node_path, direction_path):
    if not node_path or not direction_path:
        return deque(), deque()

    merged_nodes = []  # Use list instead of deque
    merged_directions = []
    
    prev_direction, prev_junctions = direction_path[0]
    current_nodes = [node_path[0]]

    for i in range(1, len(direction_path)):
        direction, junctions = direction_path[i]
        if direction == prev_direction:
            prev_junctions += junctions  # Merge junction counts
            current_nodes.append(node_path[i])  # Include node
        else:
            merged_directions.append((prev_direction, prev_junctions))
            merged_nodes.append(current_nodes[-1])  # Append last node of merged group

            # Reset
            prev_direction, prev_junctions = direction, junctions
            current_nodes = [node_path[i]]

    # Append last segment
    merged_directions.append((prev_direction, prev_junctions))
    merged_nodes.append(current_nodes[-1])

    # Convert to deque at the end (O(n))
    return deque(merged_nodes), deque(merged_directions)

if __name__ == "__main__":
    nav = Navigation()
    print(nav.dijkstra_with_directions("Start Node", "Depot 1"))
    print(nav.dijkstra_with_directions("B", "Start Node"))
    print(nav.dijkstra_with_directions("Depot 2", "D"))

# Helper function to combine adjacent nodes that have the same direction of travel
