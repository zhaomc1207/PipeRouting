from __future__ import annotations

import heapq
from dataclasses import dataclass
from math import sqrt

from .grid import Grid3D, GridIndex, build_neighbors_26, is_cell_blocked
from .io import Obstacle


@dataclass
class AStarResult:
    success: bool
    path: list[tuple[float, float, float]]
    error: str | None = None


def _euclidean(a: GridIndex, b: GridIndex) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _step_direction(a: GridIndex, b: GridIndex) -> GridIndex:
    return (b[0] - a[0], b[1] - a[1], b[2] - a[2])


def _clamp_reward(
    world_point: tuple[float, float, float],
    clamp_candidates: list[tuple[tuple[float, float, float], float]],
    clamp_weight: float,
) -> float:
    if not clamp_candidates or clamp_weight <= 0:
        return 0.0
    best = 0.0
    for pos, radius in clamp_candidates:
        if radius <= 0:
            continue
        d = sqrt(
            (world_point[0] - pos[0]) ** 2
            + (world_point[1] - pos[1]) ** 2
            + (world_point[2] - pos[2]) ** 2
        )
        if d <= radius:
            score = 1.0 - (d / radius)
            if score > best:
                best = score
    return best * clamp_weight


def astar_route(
    grid: Grid3D,
    start_world: tuple[float, float, float],
    end_world: tuple[float, float, float],
    inflated_obstacles: list[Obstacle],
    dynamic_blocks: set[GridIndex] | None = None,
    turn_penalty: float = 0.2,
    clamp_candidates: list[tuple[tuple[float, float, float], float]] | None = None,
    clamp_reward_weight: float = 0.3,
) -> AStarResult:
    start = grid.world_to_index(start_world)
    goal = grid.world_to_index(end_world)

    if not grid.in_bounds(start) or not grid.in_bounds(goal):
        return AStarResult(False, [], "Start or end point out of workspace bounds.")
    if is_cell_blocked(grid, start, inflated_obstacles, dynamic_blocks):
        return AStarResult(False, [], "Start point blocked by obstacle.")
    if is_cell_blocked(grid, goal, inflated_obstacles, dynamic_blocks):
        return AStarResult(False, [], "End point blocked by obstacle.")

    open_heap: list[tuple[float, GridIndex]] = []
    heapq.heappush(open_heap, (0.0, start))

    came_from: dict[GridIndex, GridIndex] = {}
    g_cost: dict[GridIndex, float] = {start: 0.0}
    direction_from_parent: dict[GridIndex, GridIndex | None] = {start: None}

    closed: set[GridIndex] = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path_idx = [current]
            while current in came_from:
                current = came_from[current]
                path_idx.append(current)
            path_idx.reverse()
            return AStarResult(True, [grid.index_to_world(p) for p in path_idx])

        closed.add(current)

        for nxt in build_neighbors_26(current):
            if nxt in closed:
                continue
            if is_cell_blocked(grid, nxt, inflated_obstacles, dynamic_blocks):
                continue

            step_cost = _euclidean(current, nxt) * grid.resolution
            reward = _clamp_reward(
                grid.index_to_world(nxt),
                clamp_candidates or [],
                clamp_reward_weight * grid.resolution,
            )
            # Keep every move cost positive to preserve stable A* behavior.
            adjusted_step_cost = max(step_cost * 0.1, step_cost - reward)
            tentative = g_cost[current] + adjusted_step_cost

            new_dir = _step_direction(current, nxt)
            prev_dir = direction_from_parent[current]
            if prev_dir is not None and new_dir != prev_dir:
                tentative += turn_penalty * grid.resolution

            if tentative < g_cost.get(nxt, float("inf")):
                came_from[nxt] = current
                g_cost[nxt] = tentative
                direction_from_parent[nxt] = new_dir
                f_score = tentative + _euclidean(nxt, goal) * grid.resolution
                heapq.heappush(open_heap, (f_score, nxt))

    return AStarResult(False, [], "No path found.")
