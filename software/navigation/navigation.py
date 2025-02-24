from graph import initialise_graph
import heapq
class Navigation:
    def __init__(self):
        self.graph = initialise_graph()

    def dijkstra_with_directions(self, start_node, goal_node):
        """Return (distance, node_path, dir_path)."""

        # 1) Distances dict from Node obj -> c ost
        distances = {node_obj: float("inf") for node_obj in self.graph.nodes.values()}
        distances[start_node] = 0

        # 2) came_from dict from Node obj -> (prev_node_obj, direction)
        came_from = {node_obj: None for node_obj in self.graph.nodes.values()}

        # 3) Priority queue: Store (distance, node.name, node_obj) to allow sorting
        pq = [
            (0, start_node.name, start_node)
        ]  # (distance_so_far, tie-breaker, node_obj)

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
                    heapq.heappush(
                        pq, (new_dist, neighbor_obj.name, neighbor_obj)
                    )  # Store tuple with tie-breaker

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


def combine_paths(node_path, direction_path):
    """
    Given:
      node_path: [node0, node1, node2, ..., nodeN]  (length N+1)
      direction_path: [(dir0, 1), (dir1, 1), ..., (dirN-1, 1)] (length N)
    Merge consecutive steps that have the same dir, and build a final
    list of nodes + directions.
    """
    if not node_path:
        [], []
    if len(node_path) == 1:
        # Only one node, no edges
        return node_path, node_path
    if not direction_path:
        # Means we have nodes but no directions?
        # Possibly an error or trivial path of 1 node.
        return [], []

    # Start with the first node and first direction
    merged_nodes = [node_path[0]]
    merged_directions = []

    prev_dir = direction_path[0][0]
    count_junctions = direction_path[0][1]

    # Walk through directions 1..end
    for i in range(1, len(direction_path)):
        d, c = direction_path[i]
        if d == prev_dir:
            # Same direction => merge junction counts
            count_junctions += c
        else:
            # Different direction => finalize the previous direction
            merged_directions.append((prev_dir, count_junctions))
            # The node at index i is the boundary between steps
            merged_nodes.append(node_path[i])
            # Reset direction tracking
            prev_dir = d
            count_junctions = c

    # Append the final direction
    merged_directions.append((prev_dir, count_junctions))
    # Append the final node (goal)
    merged_nodes.append(node_path[-1])

    # Reverse the lists to place the start node at the right
    merged_nodes.reverse()
    merged_directions.reverse()
    return merged_nodes, merged_directions


if __name__ == "__main__":
    nav = Navigation()
    print(
        nav.dijkstra_with_directions(
            nav.graph.get_node("Start Node"), nav.graph.get_node("Depot 1")
        )
    )
    print(
        nav.dijkstra_with_directions(
            nav.graph.get_node("B"), nav.graph.get_node("Start Node")
        )
    )
    print(
        nav.dijkstra_with_directions(
            nav.graph.get_node("Depot 2"), nav.graph.get_node("D")
        )
    )

# Helper function to combine adjacent nodes that have the same direction of travel
