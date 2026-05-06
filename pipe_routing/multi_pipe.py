from __future__ import annotations

from math import ceil, sqrt

from .astar3d import astar_route
from .collision import detect_pipe_conflicts, point_distance
from .grid import Grid3D, GridIndex, inflate_obstacle, is_cell_blocked
from .io import ClampCandidate, Pipe, RoutingCase
from .local_reroute import reroute_single_pipe_locally
from .pipe_rules import bend_count, path_length, summarize_bend_rules
from .smooth import smooth_path


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


def _path_is_collision_free(
    grid: Grid3D,
    path: list[tuple[float, float, float]],
    inflated_obstacles: list,
    dynamic_blocks: set[GridIndex],
) -> bool:
    if len(path) < 2:
        return False
    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        seg_len = sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)
        samples = max(1, int(ceil(seg_len / max(grid.resolution * 0.5, 1e-6))))
        for k in range(samples + 1):
            t = k / samples
            p = (
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            )
            if is_cell_blocked(grid, grid.world_to_index(p), inflated_obstacles, dynamic_blocks):
                return False
    return True


def _count_conflicts_with_previous(
    candidate_id: str,
    candidate_path: list[tuple[float, float, float]],
    previous_results: list[dict],
    pipe_by_id: dict[str, Pipe],
) -> int:
    routed = [p for p in previous_results if p["success"]]
    routed.append(
        {
            "id": candidate_id,
            "success": True,
            "path": [list(p) for p in candidate_path],
        }
    )
    conflicts = detect_pipe_conflicts(routed, pipe_by_id)
    return sum(1 for c in conflicts if c["pipe_a"] == candidate_id or c["pipe_b"] == candidate_id)


def _update_pipe_metrics(pipe_result: dict, clamps: list[ClampCandidate], pipe_spec: Pipe) -> None:
    path = [tuple(p) for p in pipe_result["path"]]
    pipe_result["length"] = path_length(path)
    pipe_result["bend_count"] = bend_count(path)
    bend_check = summarize_bend_rules(path, pipe_spec.min_bend_radius)
    pipe_result["min_bend_radius_required"] = bend_check["min_bend_radius_required"]
    pipe_result["min_bend_radius_observed"] = bend_check["min_bend_radius_observed"]
    pipe_result["bend_rule_violation_count"] = bend_check["bend_rule_violation_count"]
    pipe_result["bend_rule_violations"] = bend_check["bend_rule_violations"]
    # Backward-compatible fields
    pipe_result["min_bend_radius_ok"] = bend_check["min_bend_radius_ok"]
    pipe_result["bend_radius_violation_count"] = bend_check["bend_radius_violation_count"]
    pipe_result["bend_radius_violations"] = bend_check["bend_radius_violations"]
    used_clamps, min_distance_to_clamps = _compute_clamp_metrics(path, clamps)
    pipe_result["used_clamps"] = used_clamps
    pipe_result["min_distance_to_clamps"] = min_distance_to_clamps


def _select_global_paths(results: list[dict], pipe_by_id: dict[str, Pipe], clamps: list[ClampCandidate]) -> list[dict]:
    success_pipes = [p for p in results if p["success"]]
    candidates = [
        p
        for p in success_pipes
        if p.get("smoothing_applied")
        and not p.get("smoothing_reverted")
        and p.get("raw_path")
        and p.get("smoothed_path")
    ]

    if not candidates:
        return detect_pipe_conflicts(results, pipe_by_id)

    best_conflicts: list[dict] | None = None
    best_mask = 0
    best_smoothed_count = -1

    for mask in range(1 << len(candidates)):
        for idx, pipe in enumerate(candidates):
            use_smoothed = ((mask >> idx) & 1) == 1
            pipe["path"] = pipe["smoothed_path"] if use_smoothed else pipe["raw_path"]

        conflicts = detect_pipe_conflicts(results, pipe_by_id)
        smooth_count = sum(1 for idx in range(len(candidates)) if ((mask >> idx) & 1) == 1)
        if best_conflicts is None:
            best_conflicts = conflicts
            best_mask = mask
            best_smoothed_count = smooth_count
            continue

        if len(conflicts) < len(best_conflicts):
            best_conflicts = conflicts
            best_mask = mask
            best_smoothed_count = smooth_count
        elif len(conflicts) == len(best_conflicts) and smooth_count > best_smoothed_count:
            best_conflicts = conflicts
            best_mask = mask
            best_smoothed_count = smooth_count

    for idx, pipe in enumerate(candidates):
        use_smoothed = ((best_mask >> idx) & 1) == 1
        if use_smoothed:
            pipe["path"] = pipe["smoothed_path"]
            pipe["smoothing_reverted"] = False
            pipe["smoothing_revert_reason"] = None
        else:
            pipe["path"] = pipe["raw_path"]
            pipe["smoothing_reverted"] = True
            pipe["smoothing_revert_reason"] = "final_global_conflict_recheck"
        _update_pipe_metrics(pipe, clamps, pipe_by_id[pipe["id"]])

    return best_conflicts if best_conflicts is not None else detect_pipe_conflicts(results, pipe_by_id)


def route_pipes_sequentially(case: RoutingCase) -> dict:
    grid = Grid3D(case.workspace)
    dynamic_blocks: set[GridIndex] = set()
    results: list[dict] = []
    pipe_by_id = {p.id: p for p in case.pipes}

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
            clamp_reward_weight=0.0,
        )
        if not ares.success:
            results.append(
                {
                    "id": pipe.id,
                    "success": False,
                    "raw_path": [],
                    "smoothed_path": [],
                    "path": [],
                    "length": 0.0,
                    "bend_count": 0,
                    "conflict_count": 0,
                    "conflicts": [],
                    "smoothing_applied": False,
                    "smoothing_reverted": False,
                    "smoothing_revert_reason": None,
                    "used_clamps": [],
                    "min_distance_to_clamps": {},
                    "min_bend_radius_ok": True,
                    "bend_radius_violation_count": 0,
                    "bend_radius_violations": [],
                    "min_bend_radius_required": pipe.min_bend_radius,
                    "min_bend_radius_observed": float("inf"),
                    "bend_rule_violation_count": 0,
                    "bend_rule_violations": [],
                    "error": ares.error,
                    "local_reroute_applied": False,
                    "local_reroute_reason": None,
                    "rerouted": False,
                    "reroute_reason": None,
                    "affected_by_changed_region": False,
                }
            )
            continue

        raw_path = ares.path[:]
        path_smoothed = smooth_path(
            grid=grid,
            path=ares.path,
            inflated_obstacles=inflated_obstacles,
            dynamic_blocks=dynamic_blocks,
        )

        smoothing_applied = path_smoothed != raw_path
        smoothing_reverted = False
        smoothing_revert_reason = None
        final_path = path_smoothed

        if not path_smoothed:
            smoothing_reverted = smoothing_applied
            smoothing_revert_reason = "smoothed_path_empty"
            final_path = raw_path
        elif not _path_is_collision_free(grid, path_smoothed, inflated_obstacles, dynamic_blocks):
            smoothing_reverted = smoothing_applied
            smoothing_revert_reason = "smoothed_path_hits_obstacle"
            final_path = raw_path
        else:
            raw_conflicts = _count_conflicts_with_previous(pipe.id, raw_path, results, pipe_by_id)
            smooth_conflicts = _count_conflicts_with_previous(pipe.id, path_smoothed, results, pipe_by_id)
            if smooth_conflicts > raw_conflicts:
                smoothing_reverted = smoothing_applied
                smoothing_revert_reason = "smoothed_path_increases_conflicts"
                final_path = raw_path

        result = {
            "id": pipe.id,
            "success": True,
            "raw_path": [list(p) for p in raw_path],
            "smoothed_path": [list(p) for p in path_smoothed],
            "path": [list(p) for p in final_path],
            "length": 0.0,
            "bend_count": 0,
            "conflict_count": 0,
            "conflicts": [],
            "smoothing_applied": smoothing_applied,
            "smoothing_reverted": smoothing_reverted,
            "smoothing_revert_reason": smoothing_revert_reason,
            "used_clamps": [],
            "min_distance_to_clamps": {},
            "min_bend_radius_ok": True,
            "bend_radius_violation_count": 0,
            "bend_radius_violations": [],
            "min_bend_radius_required": pipe.min_bend_radius,
            "min_bend_radius_observed": float("inf"),
            "bend_rule_violation_count": 0,
            "bend_rule_violations": [],
            "error": None,
            "local_reroute_applied": False,
            "local_reroute_reason": None,
            "rerouted": False,
            "reroute_reason": None,
            "affected_by_changed_region": False,
        }
        _update_pipe_metrics(result, case.clamp_candidates, pipe)
        results.append(result)
        dynamic_blocks |= _rasterize_path_to_dynamic_blocks(grid, [tuple(p) for p in result["path"]], inflate_dist)

    conflicts = _select_global_paths(results, pipe_by_id, case.clamp_candidates)
    conflicts = _apply_local_reroute(case, grid, results, pipe_by_id, conflicts)

    for item in results:
        item["conflicts"] = []
        item["conflict_count"] = 0

    for conflict in conflicts:
        for item in results:
            if item["id"] in (conflict["pipe_a"], conflict["pipe_b"]):
                item["conflicts"].append(conflict)

    for item in results:
        item["conflict_count"] = len(item["conflicts"])

    return {"pipes": results, "conflicts": conflicts}


def _apply_local_reroute(
    case: RoutingCase,
    grid: Grid3D,
    results: list[dict],
    pipe_by_id: dict[str, Pipe],
    conflicts: list[dict],
) -> list[dict]:
    """Try local reroute only when conflict exists; accept only strict global improvement."""
    if not conflicts:
        return conflicts

    best_conflicts = conflicts
    max_rounds = 2
    for _ in range(max_rounds):
        conflict_ids = set()
        for c in best_conflicts:
            conflict_ids.add(c["pipe_a"])
            conflict_ids.add(c["pipe_b"])
        improved = False
        for pid in list(conflict_ids):
            item = next((x for x in results if x["id"] == pid and x["success"]), None)
            if item is None:
                continue
            pipe = pipe_by_id[pid]
            ok, path, err = reroute_single_pipe_locally(grid, case, pipe, results)
            if not ok or not path:
                continue

            original_path = item["path"]
            original_raw = item["raw_path"]
            original_smoothed = item["smoothed_path"]
            original_smoothing_applied = item["smoothing_applied"]
            original_smoothing_reverted = item["smoothing_reverted"]
            original_smoothing_reason = item["smoothing_revert_reason"]

            item["raw_path"] = [list(p) for p in path]
            item["smoothed_path"] = [list(p) for p in path]
            item["path"] = [list(p) for p in path]
            item["smoothing_applied"] = False
            item["smoothing_reverted"] = False
            item["smoothing_revert_reason"] = None
            item["local_reroute_applied"] = True
            item["local_reroute_reason"] = "conflict_reduction"
            _update_pipe_metrics(item, case.clamp_candidates, pipe)

            new_conflicts = detect_pipe_conflicts(results, pipe_by_id)
            if len(new_conflicts) < len(best_conflicts):
                best_conflicts = new_conflicts
                improved = True
            else:
                item["path"] = original_path
                item["raw_path"] = original_raw
                item["smoothed_path"] = original_smoothed
                item["smoothing_applied"] = original_smoothing_applied
                item["smoothing_reverted"] = original_smoothing_reverted
                item["smoothing_revert_reason"] = original_smoothing_reason
                item["local_reroute_applied"] = False
                item["local_reroute_reason"] = err or "no_improvement"
                _update_pipe_metrics(item, case.clamp_candidates, pipe)

        if not improved:
            break

    return best_conflicts
