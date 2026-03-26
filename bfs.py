from collections import deque
from maze import Node, get_neighbors, reconstruct_path


def bfs(grid, start, goal):
    """
    Breadth First Search.
    Returns (path, nodes_explored) or (None, nodes_explored) if no path.
    """
    start_node = Node(start)
    queue = deque([start_node])
    visited = {start}
    nodes_explored = 0

    while queue:
        current = queue.popleft()
        nodes_explored += 1

        if current.position == goal:
            return reconstruct_path(current), nodes_explored

        for neighbor_pos in get_neighbors(grid, current.position):
            if neighbor_pos not in visited:
                visited.add(neighbor_pos)
                queue.append(Node(neighbor_pos, parent=current, g=current.g + 1))

    return None, nodes_explored
