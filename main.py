
import heapq


def dijkstra_shortest_path(graph, start, goal):
    """
    Compute the shortest path in a graph with positive edge weights.

    graph: dict mapping node -> list of (neighbor, weight) pairs.
    start: starting node (string).
    goal: target node (string).

    Return:
        (path, total_cost)
        - path: list of nodes from start to goal with minimum total weight
        - total_cost: sum of weights along the path
        If start/goal is not in graph or goal is unreachable, return ([], None).
    """
    # This function computes the shortest path from `start` to `goal` in a
    # weighted graph with non-negative edge weights using Dijkstra's algorithm.
    # Inputs:
    # - graph: dict node -> list of (neighbor, weight)
    # - start, goal: node keys
    # Outputs:
    # - (path, total_cost) where path is a list of nodes from start to goal
    #   with minimum total weight, and total_cost is the sum of weights.
    # If start/goal not in graph or goal unreachable, return ([], None).

    # Quick edge checks
    if start not in graph or goal not in graph:
        return [], None

    # distance estimates and parent pointers
    dist = {start: 0}
    parent = {}

    # min-heap of (cost, node)
    heap = [(0, start)]

    # Standard Dijkstra loop
    while heap:
        cost_u, u = heapq.heappop(heap)

        # If this popped entry is stale (larger than known dist), skip it
        if cost_u > dist.get(u, float("inf")):
            continue

        # Early exit if we reached the goal
        if u == goal:
            break

        for v, w in graph.get(u, []):
            if w < 0:
                # Dijkstra requires non-negative weights; skip negatives
                continue
            new_cost = cost_u + w
            if new_cost < dist.get(v, float("inf")):
                dist[v] = new_cost
                parent[v] = u
                heapq.heappush(heap, (new_cost, v))

    # If goal was never reached
    if goal not in dist:
        return [], None

    # Reconstruct path from start to goal
    path = []
    node = goal
    while True:
        path.append(node)
        if node == start:
            break
        node = parent.get(node)
        if node is None:
            # No path found
            return [], None

    path.reverse()
    return path, dist[goal]


if __name__ == "__main__":
    # Optional quick check
    sample_graph = {
        "K1": [("K2", 5), ("K3", 2)],
        "K2": [("K1", 5), ("K4", 4)],
        "K3": [("K1", 2), ("K4", 7)],
        "K4": [("K2", 4), ("K3", 7)],
    }
    path, cost = dijkstra_shortest_path(sample_graph, "K1", "K4")
    print("Sample path from K1 to K4:", path, "cost:", cost)
