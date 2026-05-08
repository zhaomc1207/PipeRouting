from __future__ import annotations

import heapq
from dataclasses import dataclass
from math import acos, sqrt

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


def _direction_angle_rad(a: GridIndex, b: GridIndex) -> float:
    na = sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
    nb = sqrt(b[0] ** 2 + b[1] ** 2 + b[2] ** 2)
    if na == 0 or nb == 0:
        return 0.0
    c = max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb)))
    return acos(c)


def _distance_point_to_box_surface(point: tuple[float, float, float], obstacle: Obstacle) -> float:
    dx = 0.0
    dy = 0.0
    dz = 0.0
    if point[0] < obstacle.min_corner[0]:
        dx = obstacle.min_corner[0] - point[0]
    elif point[0] > obstacle.max_corner[0]:
        dx = point[0] - obstacle.max_corner[0]

    if point[1] < obstacle.min_corner[1]:
        dy = obstacle.min_corner[1] - point[1]
    elif point[1] > obstacle.max_corner[1]:
        dy = point[1] - obstacle.max_corner[1]

    if point[2] < obstacle.min_corner[2]:
        dz = obstacle.min_corner[2] - point[2]
    elif point[2] > obstacle.max_corner[2]:
        dz = point[2] - obstacle.max_corner[2]

    return sqrt(dx * dx + dy * dy + dz * dz)


def _clearance_penalty(
    world_point: tuple[float, float, float],
    inflated_obstacles: list[Obstacle],
    clearance_margin: float,
    clearance_penalty_weight: float,
) -> float:
    if clearance_penalty_weight <= 0 or clearance_margin <= 0 or not inflated_obstacles:
        return 0.0
    nearest = min(_distance_point_to_box_surface(world_point, obs) for obs in inflated_obstacles)
    if nearest >= clearance_margin:
        return 0.0
    return ((clearance_margin - nearest) / clearance_margin) * clearance_penalty_weight


def _conflict_penalty(
    idx: GridIndex,
    dynamic_blocks: set[GridIndex] | None,
    conflict_penalty_weight: float,
) -> float:
    if conflict_penalty_weight <= 0 or not dynamic_blocks:
        return 0.0
    if idx in dynamic_blocks:
        return conflict_penalty_weight

    x, y, z = idx
    close_count = 0
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                if (x + dx, y + dy, z + dz) in dynamic_blocks:
                    close_count += 1
    return conflict_penalty_weight * (close_count / 26.0)


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
        # Strong reward inside clamp radius, plus weaker attraction outside radius.
        if d <= radius:
            score = 1.0 - (d / radius)
        else:
            attract_dist = radius * 3.0
            if d >= attract_dist:
                continue
            score = 0.35 * (1.0 - ((d - radius) / max(attract_dist - radius, 1e-6)))
        if score > best:
            best = score
    return best * clamp_weight


def astar_route(
    grid: Grid3D,
    start_world: tuple[float, float, float],
    end_world: tuple[float, float, float],
    inflated_obstacles: list[Obstacle],
    dynamic_blocks: set[GridIndex] | None = None,
    turn_penalty: float = 0.35,
    clamp_candidates: list[tuple[tuple[float, float, float], float]] | None = None,
    clamp_reward_weight: float = 60.0,
    length_weight: float = 0.2,
    bend_smoothness_penalty: float = 0.9,
    bend_radius_penalty: float = 0.5,
    clearance_penalty: float = 0.8,
    clearance_margin_cells: float = 2.0,
    conflict_penalty: float = 0.8,
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

            raw_step_cost = _euclidean(current, nxt) * grid.resolution
            # Length is a weak objective in current business phase.
            base = raw_step_cost * max(0.05, length_weight)

            new_dir = _step_direction(current, nxt)
            prev_dir = direction_from_parent[current]
            turn_cost = 0.0
            smoothness_cost = 0.0
            radius_cost = 0.0
            if prev_dir is not None:
                angle = _direction_angle_rad(prev_dir, new_dir)
                if angle > 1e-6:
                    turn_cost = turn_penalty * grid.resolution
                    smoothness_cost = bend_smoothness_penalty * (angle / 3.1415926535) * grid.resolution
                    radius_cost = bend_radius_penalty * ((angle / 3.1415926535) ** 2) * grid.resolution

            world_nxt = grid.index_to_world(nxt)
            clear_cost = _clearance_penalty(
                world_nxt,
                inflated_obstacles,
                clearance_margin=max(grid.resolution * clearance_margin_cells, 1e-6),
                clearance_penalty_weight=clearance_penalty * grid.resolution,
            )
            conf_cost = _conflict_penalty(nxt, dynamic_blocks, conflict_penalty * grid.resolution)
            reward = _clamp_reward(
                world_nxt,
                clamp_candidates or [],
                clamp_reward_weight,
            )

            adjusted_step_cost = max(
                raw_step_cost * 0.05,
                base + turn_cost + smoothness_cost + radius_cost + clear_cost + conf_cost - reward,
            )
            tentative = g_cost[current] + adjusted_step_cost

            if tentative < g_cost.get(nxt, float("inf")):
                came_from[nxt] = current
                g_cost[nxt] = tentative
                direction_from_parent[nxt] = new_dir
                # Keep heuristic modest so path quality terms dominate over pure shortest-path tendency.
                f_score = tentative + _euclidean(nxt, goal) * grid.resolution * 0.25
                heapq.heappush(open_heap, (f_score, nxt))

    return AStarResult(False, [], "No path found.")
