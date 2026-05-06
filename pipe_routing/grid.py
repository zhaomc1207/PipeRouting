from __future__ import annotations

from dataclasses import dataclass
from math import ceil

from .io import Obstacle, Workspace

GridIndex = tuple[int, int, int]


@dataclass
class Grid3D:
    workspace: Workspace

    def __post_init__(self) -> None:
        self.min_corner = self.workspace.min_corner
        self.max_corner = self.workspace.max_corner
        self.resolution = self.workspace.resolution
        self.size_x = int(ceil((self.max_corner[0] - self.min_corner[0]) / self.resolution)) + 1
        self.size_y = int(ceil((self.max_corner[1] - self.min_corner[1]) / self.resolution)) + 1
        self.size_z = int(ceil((self.max_corner[2] - self.min_corner[2]) / self.resolution)) + 1

    def in_bounds(self, idx: GridIndex) -> bool:
        x, y, z = idx
        return 0 <= x < self.size_x and 0 <= y < self.size_y and 0 <= z < self.size_z

    def world_to_index(self, point: tuple[float, float, float]) -> GridIndex:
        return (
            int(round((point[0] - self.min_corner[0]) / self.resolution)),
            int(round((point[1] - self.min_corner[1]) / self.resolution)),
            int(round((point[2] - self.min_corner[2]) / self.resolution)),
        )

    def index_to_world(self, idx: GridIndex) -> tuple[float, float, float]:
        return (
            self.min_corner[0] + idx[0] * self.resolution,
            self.min_corner[1] + idx[1] * self.resolution,
            self.min_corner[2] + idx[2] * self.resolution,
        )


def inflate_obstacle(obstacle: Obstacle, inflate_distance: float) -> Obstacle:
    return Obstacle(
        id=obstacle.id,
        type=obstacle.type,
        min_corner=(
            obstacle.min_corner[0] - inflate_distance,
            obstacle.min_corner[1] - inflate_distance,
            obstacle.min_corner[2] - inflate_distance,
        ),
        max_corner=(
            obstacle.max_corner[0] + inflate_distance,
            obstacle.max_corner[1] + inflate_distance,
            obstacle.max_corner[2] + inflate_distance,
        ),
    )


def is_cell_in_box(grid: Grid3D, idx: GridIndex, obstacle: Obstacle) -> bool:
    x, y, z = grid.index_to_world(idx)
    return (
        obstacle.min_corner[0] <= x <= obstacle.max_corner[0]
        and obstacle.min_corner[1] <= y <= obstacle.max_corner[1]
        and obstacle.min_corner[2] <= z <= obstacle.max_corner[2]
    )


def is_cell_blocked(
    grid: Grid3D,
    idx: GridIndex,
    inflated_obstacles: list[Obstacle],
    dynamic_blocks: set[GridIndex] | None = None,
) -> bool:
    if not grid.in_bounds(idx):
        return True
    if dynamic_blocks and idx in dynamic_blocks:
        return True
    return any(is_cell_in_box(grid, idx, obs) for obs in inflated_obstacles)


def build_neighbors_26(idx: GridIndex) -> list[GridIndex]:
    x, y, z = idx
    neighbors: list[GridIndex] = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                neighbors.append((x + dx, y + dy, z + dz))
    return neighbors
