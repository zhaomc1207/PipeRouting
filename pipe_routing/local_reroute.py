from __future__ import annotations

from math import ceil, sqrt

from .astar3d import astar_route
from .collision import detect_pipe_conflicts, point_distance
from .grid import Grid3D, GridIndex, inflate_obstacle, is_cell_blocked
from .io import Pipe, RoutingCase
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


def _compute_clamp_metrics(path: list[tuple[float, float, float]], case: RoutingCase) -> tuple[list[str], dict[str, float]]:
    if not path:
        return [], {}
    used_clamps: list[str] = []
    min_distance_to_clamps: dict[str, float] = {}
    for clamp in case.clamp_candidates:
        min_d = min(point_distance(pt, clamp.position) for pt in path)
        min_distance_to_clamps[clamp.id] = min_d
        if min_d <= clamp.radius:
            used_clamps.append(clamp.id)
    return used_clamps, min_distance_to_clamps


def _build_result_record(
    pipe: Pipe,
    path: list[tuple[float, float, float]],
    rerouted: bool,
    reroute_reason: str | None,
    affected: bool,
) -> dict:
    bend = summarize_bend_rules(path, pipe.min_bend_radius)
    used_clamps, min_distance_to_clamps = ([], {})
    if path:
        # caller fills case-dependent clamp metrics
        pass
    return {
        "id": pipe.id,
        "success": bool(path),
        "raw_path": [list(p) for p in path],
        "smoothed_path": [list(p) for p in path],
        "path": [list(p) for p in path],
        "length": path_length(path),
        "bend_count": bend_count(path),
        "conflict_count": 0,
        "conflicts": [],
        "smoothing_applied": False,
        "smoothing_reverted": False,
        "smoothing_revert_reason": None,
        "used_clamps": used_clamps,
        "min_distance_to_clamps": min_distance_to_clamps,
        "min_bend_radius_ok": bend["min_bend_radius_ok"],
        "bend_radius_violation_count": bend["bend_radius_violation_count"],
        "bend_radius_violations": bend["bend_radius_violations"],
        "min_bend_radius_required": bend["min_bend_radius_required"],
        "min_bend_radius_observed": bend["min_bend_radius_observed"],
        "bend_rule_violation_count": bend["bend_rule_violation_count"],
        "bend_rule_violations": bend["bend_rule_violations"],
        "error": None if path else "No path found during local reroute.",
        "local_reroute_applied": rerouted,
        "local_reroute_reason": reroute_reason,
        "rerouted": rerouted,
        "reroute_reason": reroute_reason,
        "affected_by_changed_region": affected,
    }


def path_intersects_box(path: list[list[float]] | list[tuple[float, float, float]], box: dict, clearance: float = 0.0) -> bool:
    if not path:
        return False
    min_c = (
        float(box["min"][0]) - clearance,
        float(box["min"][1]) - clearance,
        float(box["min"][2]) - clearance,
    )
    max_c = (
        float(box["max"][0]) + clearance,
        float(box["max"][1]) + clearance,
        float(box["max"][2]) + clearance,
    )

    def inside(p: tuple[float, float, float]) -> bool:
        return min_c[0] <= p[0] <= max_c[0] and min_c[1] <= p[1] <= max_c[1] and min_c[2] <= p[2] <= max_c[2]

    pts = [tuple(float(x) for x in p) for p in path]
    for p in pts:
        if inside(p):
            return True

    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        seg_len = sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)
        samples = max(1, int(ceil(seg_len / max(1.0, clearance if clearance > 0 else 5.0))))
        for k in range(samples + 1):
            t = k / samples
            p = (
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            )
            if inside(p):
                return True
    return False


def find_affected_pipes(previous_results: dict, changed_region: dict, clearance: float = 0.0) -> list[str]:
    affected: list[str] = []
    for pipe in previous_results.get("pipes", []):
        if pipe.get("success") and path_intersects_box(pipe.get("path", []), changed_region, clearance=clearance):
            affected.append(pipe["id"])
    return affected


def local_reroute(case: RoutingCase, previous_results: dict, changed_region: dict, clearance: float = 0.0) -> dict:
    if changed_region.get("type") != "box":
        raise ValueError("changed_region.type only supports 'box' in MVP local reroute.")

    grid = Grid3D(case.workspace)
    affected_ids = set(find_affected_pipes(previous_results, changed_region, clearance=clearance))
    prev_map = {p["id"]: p for p in previous_results.get("pipes", [])}

    results: list[dict] = []
    dynamic_blocks: set[GridIndex] = set()

    # First place unaffected previous paths as fixed dynamic obstacles.
    for pipe in case.pipes:
        prev = prev_map.get(pipe.id)
        if pipe.id in affected_ids:
            continue
        if prev and prev.get("success") and prev.get("path"):
            p = [tuple(x) for x in prev["path"]]
            dynamic_blocks |= _rasterize_path_to_dynamic_blocks(grid, p, _inflate_distance(pipe))

    for pipe in case.pipes:
        prev = prev_map.get(pipe.id)
        if pipe.id not in affected_ids and prev is not None:
            rec = dict(prev)
            rec["rerouted"] = False
            rec["reroute_reason"] = None
            rec["affected_by_changed_region"] = False
            rec["local_reroute_applied"] = False
            rec["local_reroute_reason"] = None
            results.append(rec)
            continue

        inflated_obstacles = [inflate_obstacle(obs, _inflate_distance(pipe)) for obs in case.obstacles]
        ret = astar_route(
            grid=grid,
            start_world=pipe.start,
            end_world=pipe.end,
            inflated_obstacles=inflated_obstacles,
            dynamic_blocks=dynamic_blocks,
            clamp_candidates=[(c.position, c.radius) for c in case.clamp_candidates],
            clamp_reward_weight=0.0,
        )

        if not ret.success or not ret.path:
            if prev is not None:
                rec = dict(prev)
                rec["rerouted"] = False
                rec["reroute_reason"] = ret.error or "local_reroute_failed_keep_previous"
                rec["affected_by_changed_region"] = True
                rec["local_reroute_applied"] = False
                rec["local_reroute_reason"] = rec["reroute_reason"]
                results.append(rec)
                if rec.get("success"):
                    dynamic_blocks |= _rasterize_path_to_dynamic_blocks(grid, [tuple(x) for x in rec["path"]], _inflate_distance(pipe))
                continue

            rec = _build_result_record(pipe, [], False, ret.error, True)
            results.append(rec)
            continue

        raw_path = ret.path[:]
        smoothed = smooth_path(grid, raw_path, inflated_obstacles, dynamic_blocks)
        final_path = smoothed if smoothed and _path_is_collision_free(grid, smoothed, inflated_obstacles, dynamic_blocks) else raw_path
        rec = _build_result_record(pipe, final_path, True, "affected_by_changed_region", True)
        rec["raw_path"] = [list(p) for p in raw_path]
        rec["smoothed_path"] = [list(p) for p in smoothed]
        rec["path"] = [list(p) for p in final_path]
        rec["smoothing_applied"] = smoothed != raw_path
        rec["smoothing_reverted"] = final_path == raw_path and smoothed != raw_path
        rec["smoothing_revert_reason"] = "local_reroute_smoothing_safety" if rec["smoothing_reverted"] else None
        rec["local_reroute_applied"] = True
        rec["local_reroute_reason"] = "affected_by_changed_region"
        rec["rerouted"] = True
        rec["reroute_reason"] = "affected_by_changed_region"
        used_clamps, min_distance_to_clamps = _compute_clamp_metrics(final_path, case)
        rec["used_clamps"] = used_clamps
        rec["min_distance_to_clamps"] = min_distance_to_clamps
        bend = summarize_bend_rules(final_path, pipe.min_bend_radius)
        rec["min_bend_radius_ok"] = bend["min_bend_radius_ok"]
        rec["bend_radius_violation_count"] = bend["bend_radius_violation_count"]
        rec["bend_radius_violations"] = bend["bend_radius_violations"]
        rec["min_bend_radius_required"] = bend["min_bend_radius_required"]
        rec["min_bend_radius_observed"] = bend["min_bend_radius_observed"]
        rec["bend_rule_violation_count"] = bend["bend_rule_violation_count"]
        rec["bend_rule_violations"] = bend["bend_rule_violations"]
        rec["length"] = path_length(final_path)
        rec["bend_count"] = bend_count(final_path)
        results.append(rec)
        dynamic_blocks |= _rasterize_path_to_dynamic_blocks(grid, final_path, _inflate_distance(pipe))

    pipe_by_id = {p.id: p for p in case.pipes}
    conflicts = detect_pipe_conflicts(results, pipe_by_id)
    for r in results:
        r["conflicts"] = []
        r["conflict_count"] = 0
    for c in conflicts:
        for r in results:
            if r["id"] in (c["pipe_a"], c["pipe_b"]):
                r["conflicts"].append(c)
    for r in results:
        r["conflict_count"] = len(r["conflicts"])

    return {"pipes": results, "conflicts": conflicts}


# Existing helper retained for phase2 conflict-reduction hook.
def build_dynamic_blocks_for_pipe(
    grid: Grid3D,
    case: RoutingCase,
    results: list[dict],
    target_pipe: Pipe,
) -> set[GridIndex]:
    blocks: set[GridIndex] = set()
    for item in results:
        if not item.get("success") or item["id"] == target_pipe.id:
            continue
        other = next((p for p in case.pipes if p.id == item["id"]), None)
        if other is None:
            continue
        inflate_distance = (
            other.diameter / 2.0
            + target_pipe.diameter / 2.0
            + max(other.clearance, target_pipe.clearance)
        )
        radius_cells = int(round(inflate_distance / grid.resolution))
        for pt in item["path"]:
            center = grid.world_to_index((pt[0], pt[1], pt[2]))
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    for dz in range(-radius_cells, radius_cells + 1):
                        idx = (center[0] + dx, center[1] + dy, center[2] + dz)
                        if grid.in_bounds(idx):
                            blocks.add(idx)
    return blocks


def reroute_single_pipe_locally(
    grid: Grid3D,
    case: RoutingCase,
    pipe: Pipe,
    results: list[dict],
) -> tuple[bool, list[tuple[float, float, float]], str | None]:
    inflated_obstacles = [inflate_obstacle(obs, pipe.diameter / 2.0 + pipe.clearance) for obs in case.obstacles]
    dynamic_blocks = build_dynamic_blocks_for_pipe(grid, case, results, pipe)
    ret = astar_route(
        grid=grid,
        start_world=pipe.start,
        end_world=pipe.end,
        inflated_obstacles=inflated_obstacles,
        dynamic_blocks=dynamic_blocks,
        clamp_candidates=[(c.position, c.radius) for c in case.clamp_candidates],
        clamp_reward_weight=0.0,
        turn_penalty=0.4,
    )
    return ret.success, ret.path, ret.error
