from __future__ import annotations

from .astar3d import astar_route
from .collision import detect_pipe_conflicts, point_distance
from .grid import Grid3D, GridIndex, inflate_obstacle
from .io import ClampCandidate, Pipe, RoutingCase
from .pipe_rules import bend_count, path_length


def _inflate_distance(pipe: Pipe) -> float:
    return pipe.diameter / 2.0 + pipe.clearance


def _rasterize_path_to_dynamic_blocks(
    grid: Grid3D,
    path: list[tuple[float, float, float]],
    inflate_distance: float,
) -> set[GridIndex]:
    blocks: set[GridIndex] = set()
    r = int(round(inflate_distance / grid.resolution))
    for pt in path:
        center = grid.world_to_index(pt)
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    idx = (center[0] + dx, center[1] + dy, center[2] + dz)
                    if grid.in_bounds(idx):
                        blocks.add(idx)
    return blocks


def _compute_clamp_metrics(
    path: list[tuple[float, float, float]],
    clamps: list[ClampCandidate],
) -> tuple[list[str], dict[str, float]]:
    if not path:
        return [], {}

    used_clamps: list[str] = []
    min_distance_to_clamps: dict[str, float] = {}

    for clamp in clamps:
        min_d = min(point_distance(pt, clamp.position) for pt in path)
        min_distance_to_clamps[clamp.id] = min_d
        if min_d <= clamp.radius:
            used_clamps.append(clamp.id)

    return used_clamps, min_distance_to_clamps


def route_pipes_sequentially(case: RoutingCase) -> dict:
    grid = Grid3D(case.workspace)
    dynamic_blocks: set[GridIndex] = set()
    results: list[dict] = []

    for pipe in case.pipes:
        inflate_dist = _inflate_distance(pipe)
        inflated_obstacles = [inflate_obstacle(obs, inflate_dist) for obs in case.obstacles]
        clamp_candidates = [(c.position, c.radius) for c in case.clamp_candidates]
        ares = astar_route(
            grid=grid,
            start_world=pipe.start,
            end_world=pipe.end,
            inflated_obstacles=inflated_obstacles,
            dynamic_blocks=dynamic_blocks,
            clamp_candidates=clamp_candidates,
        )
        if not ares.success:
            results.append(
                {
                    "id": pipe.id,
                    "success": False,
                    "path": [],
                    "length": 0.0,
                    "bend_count": 0,
                    "conflict_count": 0,
                    "conflicts": [],
                    "used_clamps": [],
                    "min_distance_to_clamps": {},
                    "error": ares.error,
                }
            )
            continue

        length = path_length(ares.path)
        bends = bend_count(ares.path)
        used_clamps, min_distance_to_clamps = _compute_clamp_metrics(ares.path, case.clamp_candidates)
        result = {
            "id": pipe.id,
            "success": True,
            "path": [list(p) for p in ares.path],
            "length": length,
            "bend_count": bends,
            "conflict_count": 0,
            "conflicts": [],
            "used_clamps": used_clamps,
            "min_distance_to_clamps": min_distance_to_clamps,
            "error": None,
        }
        results.append(result)
        dynamic_blocks |= _rasterize_path_to_dynamic_blocks(grid, ares.path, inflate_dist)

    pipe_by_id = {p.id: p for p in case.pipes}
    conflicts = detect_pipe_conflicts(results, pipe_by_id)

    for conflict in conflicts:
        for item in results:
            if item["id"] in (conflict["pipe_a"], conflict["pipe_b"]):
                item["conflicts"].append(conflict)

    for item in results:
        item["conflict_count"] = len(item["conflicts"])

    return {"pipes": results, "conflicts": conflicts}
