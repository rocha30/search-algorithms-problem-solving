import time
import sys
import os
import math
import random
from maze import load_maze, manhattan
from bfs import bfs
from dfs import dfs
from greedy import greedy
from astar import astar


def effective_branching_factor(nodes_explored, path_length):
    """
    Estimate effective branching factor b such that:
        1 + b + b^2 + ... + b^d ≈ nodes_explored
    Solved numerically via binary search.
    Returns None if path_length <= 0.
    """
    d = path_length
    if d <= 0 or nodes_explored <= 1:
        return None

    # binary search for b in [1, nodes_explored]
    lo, hi = 1.0, float(nodes_explored)
    for _ in range(64):
        mid = (lo + hi) / 2
        try:
            if mid == 1.0:
                total = d + 1
            else:
                total = (mid ** (d + 1) - 1) / (mid - 1)
        except OverflowError:
            total = float('inf')
        if total < nodes_explored:
            lo = mid
        else:
            hi = mid
    return round((lo + hi) / 2, 4)


def run_algorithm(name, func, grid, start, goal):
    t0 = time.perf_counter()
    path, nodes_explored = func(grid, start, goal)
    elapsed = time.perf_counter() - t0

    path_length = len(path) - 1 if path else None
    bf = effective_branching_factor(nodes_explored, path_length) if path_length else None

    if path_length is not None:
        print(f"  {name:<25} path={path_length:<6}  nodes={nodes_explored:<7}  time={elapsed:.6f}s  b*={bf}")
    else:
        print(f"  {name:<25} NO PATH  nodes={nodes_explored:<7}  time={elapsed:.6f}s")

    return path, nodes_explored, elapsed, path_length, bf


def solve_maze(filepath):
    print(f"\n{'='*70}")
    print(f"Maze: {os.path.basename(filepath)}")
    grid, start, goal = load_maze(filepath)
    print(f"Size: {len(grid)}x{len(grid[0])}  Start: {start}  Goal: {goal}")
    print("-" * 70)

    run_algorithm("BFS",                lambda g, s, e: bfs(g, s, e),                   grid, start, goal)
    run_algorithm("DFS",                lambda g, s, e: dfs(g, s, e),                   grid, start, goal)
    run_algorithm("Greedy (Manhattan)", lambda g, s, e: greedy(g, s, e, 'manhattan'),   grid, start, goal)
    run_algorithm("Greedy (Euclidean)", lambda g, s, e: greedy(g, s, e, 'euclidean'),   grid, start, goal)
    run_algorithm("A* (Manhattan)",     lambda g, s, e: astar(g, s, e, 'manhattan'),    grid, start, goal)
    run_algorithm("A* (Euclidean)",     lambda g, s, e: astar(g, s, e, 'euclidean'),    grid, start, goal)


def get_free_cells(grid):
    """Return all walkable cells (value != 1) excluding start (2) and goal (3)."""
    cells = []
    for r, row in enumerate(grid):
        for c, val in enumerate(row):
            if val == 0:
                cells.append((r, c))
    return cells


def simulate_random_starts(filepath, n_starts=2, seed=None):
    print(f"\n{'#'*70}")
    print(f"RANDOM STARTS SIMULATION — {os.path.basename(filepath)}")
    grid, original_start, goal = load_maze(filepath)
    rows, cols = len(grid), len(grid[0])
    print(f"Size: {rows}x{cols}  Goal: {goal}  (seed={seed})")
    print(f"Selecting {n_starts} random starting point(s) from free cells")

    free_cells = get_free_cells(grid)
    if not free_cells:
        print("  No free cells available for random starts.")
        return

    rng = random.Random(seed)
    chosen = rng.sample(free_cells, min(n_starts, len(free_cells)))

    for i, start in enumerate(chosen, 1):
        dist = manhattan(start, goal)
        print(f"\n  {'~'*66}")
        print(f"  Random Start #{i}: {start}  |  Manhattan distance to goal: {dist}")
        print(f"  {'~'*66}")

        run_algorithm("BFS",                lambda g, s, e: bfs(g, s, e),                   grid, start, goal)
        run_algorithm("DFS",                lambda g, s, e: dfs(g, s, e),                   grid, start, goal)
        run_algorithm("Greedy (Manhattan)", lambda g, s, e: greedy(g, s, e, 'manhattan'),   grid, start, goal)
        run_algorithm("Greedy (Euclidean)", lambda g, s, e: greedy(g, s, e, 'euclidean'),   grid, start, goal)
        run_algorithm("A* (Manhattan)",     lambda g, s, e: astar(g, s, e, 'manhattan'),    grid, start, goal)
        run_algorithm("A* (Euclidean)",     lambda g, s, e: astar(g, s, e, 'euclidean'),    grid, start, goal)


def main():
    args = sys.argv[1:]

    # parse flags
    random_mode = False
    n_starts = 2
    seed = None
    filepaths = []

    i = 0
    while i < len(args):
        if args[i] == '--random':
            random_mode = True
            if i + 1 < len(args) and args[i + 1].isdigit():
                i += 1
                n_starts = int(args[i])
        elif args[i] == '--seed':
            if i + 1 < len(args):
                i += 1
                seed = int(args[i])
        else:
            filepaths.append(args[i])
        i += 1

    # if no files given, use all in data/
    if not filepaths:
        data_dir = os.path.join(os.path.dirname(__file__), "data")
        files = sorted(f for f in os.listdir(data_dir) if f.endswith(".txt"))
        if not files:
            print("No .txt files found in data/")
            return
        filepaths = [os.path.join(data_dir, f) for f in files]

    for filepath in filepaths:
        if random_mode:
            simulate_random_starts(filepath, n_starts=n_starts, seed=seed)
        else:
            solve_maze(filepath)


if __name__ == "__main__":
    main()
