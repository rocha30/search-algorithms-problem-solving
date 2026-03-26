import heapq
from maze import Node, get_neighbors, reconstruct_path, manhattan, euclidean


def greedy(grid, start, goal, heuristic='manhattan'):
    """
    Greedy Best First Search.
    Expands the node with the lowest h(n) (ignores g).
    Returns (path, nodes_explored) or (None, nodes_explored) if no path.
    """
    h_func = euclidean if heuristic == 'euclidean' else manhattan

    h0 = h_func(start, goal)
    start_node = Node(start, h=h0)

    # heap entries: (h, tie_breaker, node)
    counter = 0
    heap = [(h0, counter, start_node)]
    visited = set()
    nodes_explored = 0

    while heap:
        _, _, current = heapq.heappop(heap)

        if current.position in visited:
            continue
        visited.add(current.position)
        nodes_explored += 1

        if current.position == goal:
            return reconstruct_path(current), nodes_explored

        for neighbor_pos in get_neighbors(grid, current.position):
            if neighbor_pos not in visited:
                h = h_func(neighbor_pos, goal)
                neighbor = Node(neighbor_pos, parent=current, g=current.g + 1, h=h)
                counter += 1
                heapq.heappush(heap, (h, counter, neighbor))

    return None, nodes_explored
