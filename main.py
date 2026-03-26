import time
import sys
import os
import math
from maze import load_maze
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


def main():
    if len(sys.argv) > 1:
        for filepath in sys.argv[1:]:
            solve_maze(filepath)
    else:
        data_dir = os.path.join(os.path.dirname(__file__), "data")
        files = sorted(f for f in os.listdir(data_dir) if f.endswith(".txt"))
        if not files:
            print("No .txt files found in data/")
            return
        for fname in files:
            solve_maze(os.path.join(data_dir, fname))


if __name__ == "__main__":
    main()
