from __future__ import annotations

from copy import deepcopy
from math import acos, ceil, sqrt

from .astar3d import astar_route
from .collision import detect_pipe_conflicts, point_distance
from .grid import Grid3D, GridIndex, inflate_obstacle, is_cell_blocked
from .io import Obstacle, Pipe, RoutingCase
from .pipe_rules import bend_count, path_length, summarize_bend_rules
from .smooth import smooth_path


def make_conflict_constraint(conflict: dict, padding: float) -> dict:
    point = conflict.get("conflict_point")
    if point is None:
        point = [0.0, 0.0, 0.0]
    return {
        "id": f"constraint_{conflict['pipe_a']}_{conflict['pipe_b']}",
        "type": "box",
        "min": [point[0] - padding, point[1] - padding, point[2] - padding],
        "max": [point[0] + padding, point[1] + padding, point[2] + padding],
    }


def apply_constraints_to_case_or_planner(case: RoutingCase, constraints: list[dict]) -> list[Obstacle]:
    out = list(case.obstacles)
    for c in constraints:
        out.append(
            Obstacle(
                id=str(c["id"]),
                type="box",
                min_corner=(float(c["min"][0]), float(c["min"][1]), float(c["min"][2])),
                max_corner=(float(c["max"][0]), float(c["max"][1]), float(c["max"][2])),
            )
        )
    return out


def _path_turn_count_and_smoothness(path: list[list[float]] | list[tuple[float, float, float]]) -> tuple[int, float]:
    if len(path) < 3:
        return 0, 0.0
    turns = 0
    smoothness = 0.0
    pts = [tuple(float(c) for c in p) for p in path]
    for i in range(1, len(pts) - 1):
        a = (pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1], pts[i][2] - pts[i - 1][2])
        b = (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1], pts[i + 1][2] - pts[i][2])
        na = sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        nb = sqrt(b[0] ** 2 + b[1] ** 2 + b[2] ** 2)
        if na == 0 or nb == 0:
            continue
        c = max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb)))
        ang = acos(c)
        if ang > 1e-3:
            turns += 1
        smoothness += ang
    return turns, smoothness


def score_solution(results: list[dict], conflicts: list[dict]) -> tuple[int, int, int, int, float, float]:
    total_conflicts = len(conflicts)
    failed_pipes = sum(1 for p in results if not p.get("success"))
    total_bend_violations = sum(int(p.get("bend_rule_violation_count", 0)) for p in results if p.get("success"))
    total_turn_count = 0
    smoothness_score = 0.0
    total_length = 0.0
    for p in results:
        if not p.get("success"):
            continue
        turns, smoothness = _path_turn_count_and_smoothness(p.get("path", []))
        total_turn_count += turns
        smoothness_score += smoothness
        total_length += float(p.get("length", 0.0))
    return (
        total_conflicts,
        failed_pipes,
        total_bend_violations,
        total_turn_count,
        smoothness_score,
        total_length,
    )


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


def _path_is_collision_free(
    grid: Grid3D,
    path: list[tuple[float, float, float]],
    inflated_obstacles: list[Obstacle],
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


def _compute_clamp_metrics(path: list[tuple[float, float, float]], case: RoutingCase, pipe_id: str) -> tuple[list[str], dict[str, float]]:
    if not path:
        return [], {}
    used_clamps: list[str] = []
    min_distance_to_clamps: dict[str, float] = {}
    related = [c for c in case.clamp_candidates if not c.applies_to or pipe_id in c.applies_to]
    for clamp in related:
        min_d = min(point_distance(pt, clamp.position) for pt in path)
        min_distance_to_clamps[clamp.id] = min_d
        if min_d <= clamp.radius:
            used_clamps.append(clamp.id)
    return used_clamps, min_distance_to_clamps


def _update_metrics(item: dict, case: RoutingCase, pipe: Pipe) -> None:
    path = [tuple(p) for p in item["path"]]
    item["length"] = path_length(path)
    item["bend_count"] = bend_count(path)
    bend = summarize_bend_rules(path, pipe.min_bend_radius)
    item["min_bend_radius_ok"] = bend["min_bend_radius_ok"]
    item["bend_radius_violation_count"] = bend["bend_radius_violation_count"]
    item["bend_radius_violations"] = bend["bend_radius_violations"]
    item["min_bend_radius_required"] = bend["min_bend_radius_required"]
    item["min_bend_radius_observed"] = bend["min_bend_radius_observed"]
    item["bend_rule_violation_count"] = bend["bend_rule_violation_count"]
    item["bend_rule_violations"] = bend["bend_rule_violations"]
    used, dist = _compute_clamp_metrics(path, case, pipe.id)
    item["used_clamps"] = used
    item["min_distance_to_clamps"] = dist


def _compute_conflict_point(conflict: dict, by_id: dict[str, dict]) -> list[float] | None:
    a = by_id.get(conflict["pipe_a"])
    b = by_id.get(conflict["pipe_b"])
    if not a or not b:
        return None
    ia = int(conflict["segment_a_index"])
    ib = int(conflict["segment_b_index"])
    if ia >= len(a["path"]) - 1 or ib >= len(b["path"]) - 1:
        return None
    p1, p2 = a["path"][ia], a["path"][ia + 1]
    q1, q2 = b["path"][ib], b["path"][ib + 1]
    return [
        (p1[0] + p2[0] + q1[0] + q2[0]) / 4.0,
        (p1[1] + p2[1] + q1[1] + q2[1]) / 4.0,
        (p1[2] + p2[2] + q1[2] + q2[2]) / 4.0,
    ]


def _reroute_one_pipe_with_constraint(
    case: RoutingCase,
    results: list[dict],
    pipe_id: str,
    constraint_box: dict,
) -> list[dict] | None:
    pipe_by_id = {p.id: p for p in case.pipes}
    pipe = pipe_by_id[pipe_id]
    grid = Grid3D(case.workspace)

    dynamic_blocks: set[GridIndex] = set()
    for item in results:
        if not item.get("success") or item["id"] == pipe_id:
            continue
        other = pipe_by_id[item["id"]]
        inflate_distance = (
            other.diameter / 2.0
            + pipe.diameter / 2.0
            + max(other.clearance, pipe.clearance)
        )
        dynamic_blocks |= _rasterize_path_to_dynamic_blocks(grid, [tuple(x) for x in item["path"]], inflate_distance)

    obstacles = apply_constraints_to_case_or_planner(case, [constraint_box])
    inflated_obstacles = [inflate_obstacle(obs, _inflate_distance(pipe)) for obs in obstacles]
    ret = astar_route(
        grid=grid,
        start_world=pipe.start,
        end_world=pipe.end,
        inflated_obstacles=inflated_obstacles,
        dynamic_blocks=dynamic_blocks,
        clamp_candidates=[(c.position, c.radius) for c in case.clamp_candidates if not c.applies_to or pipe.id in c.applies_to],
        clamp_reward_weight=35.0,
    )
    if not ret.success or not ret.path:
        return None

    raw = ret.path[:]
    own_clamps = [c for c in case.clamp_candidates if not c.applies_to or pipe.id in c.applies_to]
    protected: list[tuple[float, float, float]] = []
    for c in own_clamps:
        best = None
        best_d = float("inf")
        for p in raw:
            d = point_distance(p, c.position)
            if d <= c.radius and d < best_d:
                best_d = d
                best = p
        if best is not None:
            protected.append(best)
    smoothed = smooth_path(grid, raw, inflated_obstacles, dynamic_blocks, protected_points=protected)
    final = smoothed if smoothed and _path_is_collision_free(grid, smoothed, inflated_obstacles, dynamic_blocks) else raw

    out = deepcopy(results)
    target = next(p for p in out if p["id"] == pipe_id)
    target["raw_path"] = [list(p) for p in raw]
    target["smoothed_path"] = [list(p) for p in smoothed]
    target["path"] = [list(p) for p in final]
    target["smoothing_applied"] = smoothed != raw
    target["smoothing_reverted"] = final == raw and smoothed != raw
    target["smoothing_revert_reason"] = "cbs_smoothing_safety" if target["smoothing_reverted"] else None
    target["cbs_rerouted"] = True
    target["cbs_constraints_count"] = int(target.get("cbs_constraints_count", 0)) + 1
    target["cbs_reroute_reason"] = "constraint_box_reroute"
    _update_metrics(target, case, pipe)
    return out


def simplified_cbs(case: RoutingCase, initial_results: dict, max_iterations: int = 5, padding: float = 40.0) -> dict:
    results = deepcopy(initial_results["pipes"])
    pipe_by_id = {p.id: p for p in case.pipes}

    for item in results:
        item.setdefault("cbs_rerouted", False)
        item.setdefault("cbs_constraints_count", 0)
        item.setdefault("cbs_reroute_reason", None)

    conflicts = detect_pipe_conflicts(results, pipe_by_id)
    cbs_log: list[str] = []
    if not conflicts:
        cbs_log.append("No conflicts detected; CBS exits immediately.")
        return {
            "pipes": results,
            "conflicts": conflicts,
            "cbs_enabled": True,
            "cbs_iterations": 0,
            "cbs_resolved_conflicts": 0,
            "cbs_remaining_conflicts": 0,
            "cbs_log": cbs_log,
        }

    initial_conflicts_count = len(conflicts)
    iterations = 0
    for it in range(max_iterations):
        if not conflicts:
            break
        iterations += 1
        by_id = {p["id"]: p for p in results}
        conflict = dict(conflicts[0])
        conflict["conflict_point"] = _compute_conflict_point(conflict, by_id)
        constraint = make_conflict_constraint(conflict, padding)

        candidates: list[tuple[tuple[int, int, int, int, float, float], list[dict], str]] = []
        for pid in (conflict["pipe_a"], conflict["pipe_b"]):
            rerouted = _reroute_one_pipe_with_constraint(case, results, pid, constraint)
            if rerouted is None:
                cbs_log.append(f"iter={it+1}: candidate {pid} failed to reroute")
                continue
            cand_conflicts = detect_pipe_conflicts(rerouted, pipe_by_id)
            score = score_solution(rerouted, cand_conflicts)
            candidates.append((score, rerouted, pid))
            cbs_log.append(f"iter={it+1}: candidate {pid} score={score}")

        if not candidates:
            cbs_log.append(f"iter={it+1}: no feasible candidate, stop.")
            break

        candidates.sort(key=lambda x: x[0])
        best_score, best_results, best_pid = candidates[0]
        current_score = score_solution(results, conflicts)
        if best_score < current_score:
            results = best_results
            conflicts = detect_pipe_conflicts(results, pipe_by_id)
            cbs_log.append(f"iter={it+1}: accepted reroute on {best_pid}, new_conflicts={len(conflicts)}")
        else:
            cbs_log.append(f"iter={it+1}: no improvement, stop.")
            break

    remaining = len(conflicts)
    return {
        "pipes": results,
        "conflicts": conflicts,
        "cbs_enabled": True,
        "cbs_iterations": iterations,
        "cbs_resolved_conflicts": max(0, initial_conflicts_count - remaining),
        "cbs_remaining_conflicts": remaining,
        "cbs_log": cbs_log,
    }
