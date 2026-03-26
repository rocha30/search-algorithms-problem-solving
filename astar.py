import heapq
from maze import Node, get_neighbors, reconstruct_path, manhattan, euclidean


def astar(grid, start, goal, heuristic='manhattan'):
    """
    A* Search.
    Expands the node with the lowest f(n) = g(n) + h(n).
    Returns (path, nodes_explored) or (None, nodes_explored) if no path.
    """
    h_func = euclidean if heuristic == 'euclidean' else manhattan

    h0 = h_func(start, goal)
    start_node = Node(start, g=0, h=h0)

    # heap entries: (f, tie_breaker, node)
    counter = 0
    heap = [(start_node.f, counter, start_node)]

    # best known g for each position
    best_g = {start: 0}
    nodes_explored = 0

    while heap:
        _, _, current = heapq.heappop(heap)

        # skip if we already found a better path to this position
        if current.g > best_g.get(current.position, float('inf')):
            continue

        nodes_explored += 1

        if current.position == goal:
            return reconstruct_path(current), nodes_explored

        for neighbor_pos in get_neighbors(grid, current.position):
            new_g = current.g + 1
            if new_g < best_g.get(neighbor_pos, float('inf')):
                best_g[neighbor_pos] = new_g
                h = h_func(neighbor_pos, goal)
                neighbor = Node(neighbor_pos, parent=current, g=new_g, h=h)
                counter += 1
                heapq.heappush(heap, (neighbor.f, counter, neighbor))

    return None, nodes_explored
