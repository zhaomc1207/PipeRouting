from __future__ import annotations

from math import ceil, sqrt

from .grid import Grid3D, GridIndex, is_cell_blocked
from .io import Obstacle

Vec3 = tuple[float, float, float]


def _segment_length(a: Vec3, b: Vec3) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _can_connect_directly(
    grid: Grid3D,
    a: Vec3,
    b: Vec3,
    inflated_obstacles: list[Obstacle],
    dynamic_blocks: set[GridIndex] | None,
) -> bool:
    seg_len = _segment_length(a, b)
    if seg_len == 0:
        return True

    # Sample along segment at half-cell spacing to avoid crossing blocked cells.
    step = max(grid.resolution * 0.5, 1e-6)
    n = max(1, int(ceil(seg_len / step)))
    for i in range(n + 1):
        t = i / n
        p = (
            a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t,
        )
        idx = grid.world_to_index(p)
        if is_cell_blocked(grid, idx, inflated_obstacles, dynamic_blocks):
            return False
    return True


def smooth_path(
    grid: Grid3D,
    path: list[Vec3],
    inflated_obstacles: list[Obstacle],
    dynamic_blocks: set[GridIndex] | None = None,
) -> list[Vec3]:
    """Shortcut path while preserving obstacle safety."""
    if len(path) <= 2:
        return path[:]

    smoothed: list[Vec3] = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        farthest = anchor + 1
        for j in range(len(path) - 1, anchor, -1):
            if _can_connect_directly(
                grid=grid,
                a=path[anchor],
                b=path[j],
                inflated_obstacles=inflated_obstacles,
                dynamic_blocks=dynamic_blocks,
            ):
                farthest = j
                break
        smoothed.append(path[farthest])
        anchor = farthest

    return smoothed

