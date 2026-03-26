import math


class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position  # (row, col)
        self.parent = parent
        self.g = g  # cost from start
        self.h = h  # heuristic estimate to goal

    @property
    def f(self):
        return self.g + self.h

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash(self.position)


def load_maze(filepath):
    """Load maze from .txt file.
    Supports two formats:
      - Comma-separated: 0,1,1,2,...
      - Concatenated:    011120...
    Returns (grid, start, goal).
    """
    grid = []
    start = None
    goal = None

    with open(filepath, 'r') as f:
        raw_lines = [l.rstrip('\n') for l in f if l.strip()]

    # detect format from first line
    csv_format = ',' in raw_lines[0]

    for r, line in enumerate(raw_lines):
        if csv_format:
            tokens = line.split(',')
        else:
            tokens = list(line)

        row = []
        for c, token in enumerate(tokens):
            token = token.strip()
            if token not in ('0', '1', '2', '3'):
                continue
            val = int(token)
            row.append(val)
            if val == 2:
                start = (r, len(row) - 1)
            elif val == 3:
                goal = (r, len(row) - 1)
        if row:
            grid.append(row)

    return grid, start, goal


def get_neighbors(grid, position):
    """Return valid neighbors in order: Up, Right, Down, Left."""
    row, col = position
    rows = len(grid)
    cols = len(grid[0])
    moves = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # Up, Right, Down, Left
    neighbors = []
    for dr, dc in moves:
        nr, nc = row + dr, col + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != 1:
            neighbors.append((nr, nc))
    return neighbors


def reconstruct_path(node):
    """Trace back from goal node to start. Returns list of positions."""
    path = []
    current = node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return list(reversed(path))


def manhattan(pos, goal):
    return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])


def euclidean(pos, goal):
    return math.sqrt((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2)
