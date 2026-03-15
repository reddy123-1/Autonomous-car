"""
Grid map representation for pathfinding.
Each cell is either free (0) or occupied (1). World coordinates in cm; origin at (0,0).
"""

from typing import List, Tuple, Optional

import numpy as np

from config import GRID_WIDTH, GRID_HEIGHT, CELL_SIZE_CM


class GridMap:
    """
    2D grid map for A* and navigation. 0 = free, 1 = obstacle.
    Origin (0,0) at bottom-left; x increases right, y increases up.
    Indices: grid[gy, gx] with gy = row (y), gx = column (x).
    """

    def __init__(
        self,
        width: Optional[int] = None,
        height: Optional[int] = None,
        cell_size_cm: Optional[float] = None,
    ) -> None:
        self.width = width if width is not None else GRID_WIDTH
        self.height = height if height is not None else GRID_HEIGHT
        self.cell_size_cm = cell_size_cm if cell_size_cm is not None else CELL_SIZE_CM
        self._grid = np.zeros((self.height, self.width), dtype=np.uint8)

    def set_obstacle(self, gx: int, gy: int) -> None:
        """Mark cell (gx, gy) as obstacle. Bounds-checked."""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self._grid[gy, gx] = 1

    def clear_obstacle(self, gx: int, gy: int) -> None:
        """Clear cell (gx, gy) to free. Bounds-checked."""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self._grid[gy, gx] = 0

    def set_obstacle_rect(self, gx0: int, gy0: int, gx1: int, gy1: int) -> None:
        """Set all cells in rectangle [gx0..gx1] x [gy0..gy1] (inclusive) as obstacle."""
        for gx in range(max(0, gx0), min(self.width, gx1 + 1)):
            for gy in range(max(0, gy0), min(self.height, gy1 + 1)):
                self._grid[gy, gx] = 1

    def clear_all(self) -> None:
        """Set all cells to free."""
        self._grid.fill(0)

    def is_free(self, gx: int, gy: int) -> bool:
        """True if cell is inside bounds and not occupied."""
        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return False
        return self._grid[gy, gx] == 0

    def in_bounds(self, gx: int, gy: int) -> bool:
        """True if (gx, gy) is inside grid bounds."""
        return 0 <= gx < self.width and 0 <= gy < self.height

    def get_grid(self) -> np.ndarray:
        """Return the 2D array (height, width). 0=free, 1=obstacle."""
        return self._grid

    def world_to_grid(self, x_cm: float, y_cm: float) -> Tuple[int, int]:
        """
        Convert world position (cm) to grid cell (gx, gy).
        Assumes world origin (0,0) at grid (0,0) cell center.
        """
        gx = int(round(x_cm / self.cell_size_cm))
        gy = int(round(y_cm / self.cell_size_cm))
        return (gx, gy)

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid cell to world position (center of cell) in cm."""
        x_cm = (gx + 0.5) * self.cell_size_cm
        y_cm = (gy + 0.5) * self.cell_size_cm
        return (x_cm, y_cm)

    def get_neighbors(
        self,
        gx: int,
        gy: int,
        allow_diagonal: bool = False,
    ) -> List[Tuple[int, int]]:
        """
        Return list of (gx, gy) neighbors that are free and in bounds.
        4-connected if allow_diagonal=False; 8-connected if True.
        """
        deltas: List[Tuple[int, int]] = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
        ]
        if allow_diagonal:
            deltas.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])
        out: List[Tuple[int, int]] = []
        for dx, dy in deltas:
            nx, ny = gx + dx, gy + dy
            if self.is_free(nx, ny):
                out.append((nx, ny))
        return out

    def copy(self) -> "GridMap":
        """Return a deep copy of this grid map."""
        other = GridMap(self.width, self.height, self.cell_size_cm)
        other._grid = self._grid.copy()
        return other
