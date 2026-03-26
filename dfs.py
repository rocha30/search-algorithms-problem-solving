from maze import Node, get_neighbors, reconstruct_path


def dfs(grid, start, goal):
    """
    Depth First Search (iterative).
    Returns (path, nodes_explored) or (None, nodes_explored) if no path.
    """
    start_node = Node(start)
    stack = [start_node]
    visited = set()
    nodes_explored = 0

    while stack:
        current = stack.pop()

        if current.position in visited:
            continue
        visited.add(current.position)
        nodes_explored += 1

        if current.position == goal:
            return reconstruct_path(current), nodes_explored

        for neighbor_pos in get_neighbors(grid, current.position):
            if neighbor_pos not in visited:
                stack.append(Node(neighbor_pos, parent=current, g=current.g + 1))

    return None, nodes_explored
