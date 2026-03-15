"""
A* pathfinding on a grid map.
Produces a path from start to goal as list of (gx, gy) cells. Cost: 1 for 4-neighbor, sqrt(2) for diagonal.
"""

import heapq
from typing import List, Tuple, Optional

from .grid_map import GridMap
from utils.logger import log


def _heuristic(gx1: int, gy1: int, gx2: int, gy2: int) -> float:
    """Euclidean heuristic (admissible for 8-connected)."""
    return ((gx1 - gx2) ** 2 + (gy1 - gy2) ** 2) ** 0.5


def _move_cost(gx0: int, gy0: int, gx1: int, gy1: int) -> float:
    """Cost of moving from (gx0, gy0) to (gx1, gy1). 1 for cardinal, sqrt(2) for diagonal."""
    dx = abs(gx1 - gx0)
    dy = abs(gy1 - gy0)
    if dx + dy == 2:
        return 1.414213562  # sqrt(2)
    return 1.0


def astar(
    grid: GridMap,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    allow_diagonal: bool = False,
) -> List[Tuple[int, int]]:
    """
    A* path from start to goal on the given grid.

    Args:
        grid: GridMap instance (0=free, 1=obstacle).
        start: (gx, gy) start cell.
        goal: (gx, gy) goal cell.
        allow_diagonal: If True, 8-connected; else 4-connected.

    Returns:
        List of (gx, gy) from start to goal (inclusive). Empty if no path.
    """
    sx, sy = start
    gx, gy = goal
    if not grid.is_free(sx, sy):
        log.warning("A* start (%s, %s) not free", sx, sy)
        return []
    if not grid.is_free(gx, gy):
        log.warning("A* goal (%s, %s) not free", gx, gy)
        return []

    # (f, tie_breaker, g, (cx, cy), path)
    tie = 0
    open_set: List[Tuple[float, int, float, Tuple[int, int], List[Tuple[int, int]]]] = [
        (0.0, tie, 0.0, (sx, sy), [(sx, sy)])
    ]
    closed: set = set()

    while open_set:
        f, _, g, (cx, cy), path = heapq.heappop(open_set)
        if (cx, cy) in closed:
            continue
        closed.add((cx, cy))
        if (cx, cy) == (gx, gy):
            log.info("A* path found, length=%d cells", len(path))
            return path

        for (nx, ny) in grid.get_neighbors(cx, cy, allow_diagonal):
            if (nx, ny) in closed:
                continue
            cost = _move_cost(cx, cy, nx, ny)
            g_new = g + cost
            h = _heuristic(nx, ny, gx, gy)
            f_new = g_new + h
            tie += 1
            heapq.heappush(
                open_set,
                (f_new, tie, g_new, (nx, ny), path + [(nx, ny)]),
            )

    log.warning("A* no path from %s to %s", start, goal)
    return []


def path_to_waypoints_cm(grid: GridMap, path: List[Tuple[int, int]]) -> List[Tuple[float, float]]:
    """
    Convert A* path (list of (gx, gy)) to waypoints in world cm (cell centers).

    Returns:
        List of (x_cm, y_cm) waypoints.
    """
    return [grid.grid_to_world(px, py) for px, py in path]


def simplify_path(path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """
    Remove collinear intermediate points (optional). Keeps start and goal.
    Returns new path with only direction-change points.
    """
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    for i in range(1, len(path) - 1):
        px, py = path[i - 1]
        cx, cy = path[i]
        nx, ny = path[i + 1]
        # Same direction if (cx-px, cy-py) proportional to (nx-cx, ny-cy)
        dx1, dy1 = cx - px, cy - py
        dx2, dy2 = nx - cx, ny - cy
        if dx1 * dy2 != dy1 * dx2:  # direction change
            out.append((cx, cy))
    out.append(path[-1])
    return out
