from __future__ import annotations

from math import ceil, sqrt

from .astar3d import astar_route
from .cbs_fallback import apply_simplified_cbs_fallback
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
    protected_cells: set[GridIndex] | None = None,
) -> set[GridIndex]:
    blocks: set[GridIndex] = set()
    r = int(round(inflate_distance / grid.resolution))
    protected = protected_cells or set()
    for pt in path:
        center = grid.world_to_index(pt)
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    idx = (center[0] + dx, center[1] + dy, center[2] + dz)
                    if grid.in_bounds(idx) and idx not in protected:
                        blocks.add(idx)
    return blocks


def _collect_terminal_allowance_cells(grid: Grid3D, pipes: list[Pipe], radius_cells: int = 1) -> set[GridIndex]:
    cells: set[GridIndex] = set()
    for p in pipes:
        for world_pt in (p.start, p.end):
            c = grid.world_to_index(world_pt)
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    for dz in range(-radius_cells, radius_cells + 1):
                        idx = (c[0] + dx, c[1] + dy, c[2] + dz)
                        if grid.in_bounds(idx):
                            cells.add(idx)
    return cells


def _pipe_difficulty_score(case: RoutingCase, pipe: Pipe) -> tuple[int, int, float]:
    grid = Grid3D(case.workspace)
    inflate_dist = _inflate_distance(pipe)
    inflated = [inflate_obstacle(obs, inflate_dist) for obs in case.obstacles]
    start_idx = grid.world_to_index(pipe.start)
    end_idx = grid.world_to_index(pipe.end)
    blocked_neighbors = 0
    for c in (start_idx, end_idx):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    n = (c[0] + dx, c[1] + dy, c[2] + dz)
                    if is_cell_blocked(grid, n, inflated, dynamic_blocks=None):
                        blocked_neighbors += 1
    length_hint = point_distance(pipe.start, pipe.end)
    return (blocked_neighbors, int(pipe.diameter + pipe.clearance), length_hint)


def _compute_clamp_metrics(
    path: list[tuple[float, float, float]],
    clamps: list[ClampCandidate],
    pipe_id: str,
) -> tuple[list[str], dict[str, float]]:
    if not path:
        return [], {}

    related_clamps = [c for c in clamps if not c.applies_to or pipe_id in c.applies_to]
    used_clamps: list[str] = []
    min_distance_to_clamps: dict[str, float] = {}

    for clamp in related_clamps:
        min_d = min(point_distance(pt, clamp.position) for pt in path)
        min_distance_to_clamps[clamp.id] = min_d
        if min_d <= clamp.radius:
            used_clamps.append(clamp.id)

    return used_clamps, min_distance_to_clamps


def _pipe_own_clamps(clamps: list[ClampCandidate], pipe_id: str) -> list[ClampCandidate]:
    return [c for c in clamps if not c.applies_to or pipe_id in c.applies_to]


def _clamp_hit_points(path: list[tuple[float, float, float]], clamps: list[ClampCandidate]) -> list[tuple[float, float, float]]:
    hits: list[tuple[float, float, float]] = []
    if not path:
        return hits
    for clamp in clamps:
        best_p: tuple[float, float, float] | None = None
        best_d = float("inf")
        for p in path:
            d = point_distance(p, clamp.position)
            if d <= clamp.radius and d < best_d:
                best_d = d
                best_p = p
        if best_p is not None:
            hits.append(best_p)
    return hits


def _distance_point_to_segment(p: tuple[float, float, float], a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ap = (p[0] - a[0], p[1] - a[1], p[2] - a[2])
    ab2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]
    if ab2 <= 1e-9:
        return point_distance(p, a)
    t = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / ab2
    t = max(0.0, min(1.0, t))
    q = (a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t)
    return point_distance(p, q)


def _build_clamp_candidate_sets(pipe: Pipe, own_clamps: list[ClampCandidate]) -> list[list[tuple[tuple[float, float, float], float]]]:
    if not own_clamps:
        return [[]]

    ranked = sorted(
        own_clamps,
        key=lambda c: (
            _distance_point_to_segment(c.position, pipe.start, pipe.end),
            point_distance(pipe.start, c.position) + point_distance(c.position, pipe.end),
        ),
    )

    sets: list[list[tuple[tuple[float, float, float], float]]] = []
    seen: set[tuple[str, ...]] = set()

    def _add(clamps: list[ClampCandidate]) -> None:
        key = tuple(sorted(c.id for c in clamps))
        if key in seen:
            return
        seen.add(key)
        sets.append([(c.position, c.radius) for c in clamps])

    _add(ranked)
    top_n = min(4, len(ranked))
    for i in range(top_n):
        _add([ranked[i]])
    for i in range(top_n):
        for j in range(i + 1, top_n):
            _add([ranked[i], ranked[j]])
    return sets if sets else [[]]


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


def _validate_pipe_endpoints(case: RoutingCase) -> dict[str, list[str]]:
    grid = Grid3D(case.workspace)
    issues: dict[str, list[str]] = {}
    for pipe in case.pipes:
        inflate_dist = _inflate_distance(pipe)
        inflated_obstacles = [inflate_obstacle(obs, inflate_dist) for obs in case.obstacles]
        errors: list[str] = []
        start_idx = grid.world_to_index(pipe.start)
        end_idx = grid.world_to_index(pipe.end)
        if not grid.in_bounds(start_idx):
            errors.append("start_out_of_workspace")
        elif is_cell_blocked(grid, start_idx, inflated_obstacles, dynamic_blocks=None):
            errors.append("start_blocked_by_inflated_obstacle")
        if not grid.in_bounds(end_idx):
            errors.append("end_out_of_workspace")
        elif is_cell_blocked(grid, end_idx, inflated_obstacles, dynamic_blocks=None):
            errors.append("end_blocked_by_inflated_obstacle")
        if errors:
            issues[pipe.id] = errors
    return issues


def _find_nearest_free_index(
    grid: Grid3D,
    source_idx: GridIndex,
    inflated_obstacles: list,
    max_radius: int = 8,
) -> GridIndex | None:
    sx, sy, sz = source_idx
    best: GridIndex | None = None
    best_d2 = float("inf")
    for r in range(0, max_radius + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    if max(abs(dx), abs(dy), abs(dz)) != r:
                        continue
                    idx = (sx + dx, sy + dy, sz + dz)
                    if not grid.in_bounds(idx):
                        continue
                    if is_cell_blocked(grid, idx, inflated_obstacles, dynamic_blocks=None):
                        continue
                    d2 = dx * dx + dy * dy + dz * dz
                    if d2 < best_d2:
                        best_d2 = d2
                        best = idx
        if best is not None:
            return best
    return None


def _sanitize_case_pipe_endpoints(case: RoutingCase) -> tuple[RoutingCase, dict[str, list[str]]]:
    grid = Grid3D(case.workspace)
    issues = _validate_pipe_endpoints(case)
    if not issues:
        return case, {}

    repaired_pipes: list[Pipe] = []
    unresolved: dict[str, list[str]] = {}
    for pipe in case.pipes:
        inflate_dist = _inflate_distance(pipe)
        inflated_obstacles = [inflate_obstacle(obs, inflate_dist) for obs in case.obstacles]
        start_idx = grid.world_to_index(pipe.start)
        end_idx = grid.world_to_index(pipe.end)
        new_start = pipe.start
        new_end = pipe.end
        reasons = issues.get(pipe.id, [])

        if any(x.startswith("start_") for x in reasons):
            free_start = _find_nearest_free_index(grid, start_idx, inflated_obstacles)
            if free_start is not None:
                new_start = grid.index_to_world(free_start)
            else:
                unresolved.setdefault(pipe.id, []).append("start_no_nearby_free_space")
        if any(x.startswith("end_") for x in reasons):
            free_end = _find_nearest_free_index(grid, end_idx, inflated_obstacles)
            if free_end is not None:
                new_end = grid.index_to_world(free_end)
            else:
                unresolved.setdefault(pipe.id, []).append("end_no_nearby_free_space")

        repaired_pipes.append(
            Pipe(
                id=pipe.id,
                start=new_start,
                end=new_end,
                diameter=pipe.diameter,
                clearance=pipe.clearance,
                min_bend_radius=pipe.min_bend_radius,
            )
        )

    repaired_case = RoutingCase(
        workspace=case.workspace,
        obstacles=case.obstacles,
        pipes=repaired_pipes,
        clamp_candidates=case.clamp_candidates,
    )
    # Re-check repaired endpoints; any remaining issue is unresolved input invalid.
    remaining = _validate_pipe_endpoints(repaired_case)
    for pid, rs in remaining.items():
        unresolved.setdefault(pid, [])
        for r in rs:
            if r not in unresolved[pid]:
                unresolved[pid].append(r)
    return repaired_case, unresolved


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
    own_clamps = _pipe_own_clamps(clamps, pipe_spec.id)
    used_clamps, min_distance_to_clamps = _compute_clamp_metrics(path, clamps, pipe_spec.id)
    own_ids = [c.id for c in own_clamps]
    pipe_result["used_clamps"] = used_clamps
    pipe_result["min_distance_to_clamps"] = min_distance_to_clamps
    pipe_result["own_clamps"] = own_ids
    pipe_result["missed_clamps"] = [cid for cid in own_ids if cid not in used_clamps]


def _evaluate_path_variant(
    path: list[tuple[float, float, float]],
    pipe: Pipe,
    previous_results: list[dict],
    pipe_by_id: dict[str, Pipe],
    clamps: list[ClampCandidate],
) -> tuple[int, int, int, int, int, float]:
    bend_info = summarize_bend_rules(path, pipe.min_bend_radius)
    violations = int(bend_info["bend_rule_violation_count"])
    conflicts = _count_conflicts_with_previous(pipe.id, path, previous_results, pipe_by_id)
    used_clamps, _ = _compute_clamp_metrics(path, clamps, pipe.id)
    # priority: bend violations -> conflicts -> bend_count -> smoothness -> clamp usage -> length
    return (
        violations,
        conflicts,
        bend_count(path),
        len(path),
        -len(used_clamps),
        path_length(path),
    )


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

    def _global_score(conflicts: list[dict]) -> tuple[int, int, int, int, int, float, int]:
        total_conflicts = len(conflicts)
        total_bend_violations = 0
        min_bend_radius_bad_count = 0
        total_bends = 0
        total_missed_clamps = 0
        total_length = 0.0
        for item in results:
            if not item.get("success"):
                continue
            pid = item["id"]
            path = [tuple(p) for p in item.get("path", [])]
            if len(path) < 2:
                continue
            bend = summarize_bend_rules(path, pipe_by_id[pid].min_bend_radius)
            violations = int(bend["bend_rule_violation_count"])
            total_bend_violations += violations
            if violations > 0:
                min_bend_radius_bad_count += 1
            total_bends += bend_count(path)
            used, _ = _compute_clamp_metrics(path, clamps, pid)
            own = _pipe_own_clamps(clamps, pid)
            total_missed_clamps += max(0, len(own) - len(used))
            total_length += path_length(path)
        return (
            total_conflicts,
            total_bend_violations,
            min_bend_radius_bad_count,
            total_bends,
            total_missed_clamps,
            total_length,
            -sum(1 for p in candidates if p["path"] == p["smoothed_path"]),
        )

    # Scalable global selection:
    # start with all-smoothed and greedily accept per-pipe toggles (raw/smoothed)
    # only if global score strictly improves.
    selected_smoothed: dict[str, bool] = {}
    for pipe in candidates:
        pipe["path"] = pipe["smoothed_path"]
        selected_smoothed[pipe["id"]] = True

    best_conflicts = detect_pipe_conflicts(results, pipe_by_id)
    best_score = _global_score(best_conflicts)

    improved = True
    max_rounds = 3
    rounds = 0
    while improved and rounds < max_rounds:
        rounds += 1
        improved = False
        for pipe in candidates:
            pid = pipe["id"]
            current_state = selected_smoothed[pid]
            pipe["path"] = pipe["raw_path"] if current_state else pipe["smoothed_path"]
            trial_conflicts = detect_pipe_conflicts(results, pipe_by_id)
            trial_score = _global_score(trial_conflicts)
            if trial_score < best_score:
                best_score = trial_score
                best_conflicts = trial_conflicts
                selected_smoothed[pid] = not current_state
                improved = True
            else:
                pipe["path"] = pipe["smoothed_path"] if current_state else pipe["raw_path"]

    for pipe in candidates:
        use_smoothed = selected_smoothed.get(pipe["id"], True)
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
    original_issues = _validate_pipe_endpoints(case)
    case, unresolved_issues = _sanitize_case_pipe_endpoints(case)
    grid = Grid3D(case.workspace)
    dynamic_blocks: set[GridIndex] = set()
    results: list[dict] = []
    ordered_pipes = sorted(case.pipes, key=lambda p: _pipe_difficulty_score(case, p), reverse=True)
    pipe_by_id = {p.id: p for p in ordered_pipes}
    terminal_allowance = _collect_terminal_allowance_cells(grid, ordered_pipes, radius_cells=1)
    endpoint_issues = unresolved_issues

    for pipe in ordered_pipes:
        inflate_dist = _inflate_distance(pipe)
        inflated_obstacles = [inflate_obstacle(obs, inflate_dist) for obs in case.obstacles]
        own_clamps = _pipe_own_clamps(case.clamp_candidates, pipe.id)
        clamp_candidate_sets = _build_clamp_candidate_sets(pipe, own_clamps)
        attempts = [
            {
                "name": "default_clamp_guided",
                "clamp_reward_weight": 12.0,
                "turn_penalty": 0.8,
                "bend_smoothness_penalty": 1.4,
                "bend_radius_penalty": 1.2,
                "length_weight": 0.15,
            },
            {
                "name": "reduced_clamp",
                "clamp_reward_weight": 4.0,
                "turn_penalty": 0.95,
                "bend_smoothness_penalty": 1.6,
                "bend_radius_penalty": 1.5,
                "length_weight": 0.15,
            },
            {
                "name": "no_clamp_bend_safe",
                "clamp_reward_weight": 0.0,
                "turn_penalty": 1.1,
                "bend_smoothness_penalty": 2.0,
                "bend_radius_penalty": 1.9,
                "length_weight": 0.12,
            },
        ]

        best_choice: dict | None = None
        last_error: str | None = None
        if pipe.id in endpoint_issues:
            reasons = endpoint_issues[pipe.id]
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
                    "own_clamps": [c.id for c in own_clamps],
                    "missed_clamps": [c.id for c in own_clamps],
                    "min_bend_radius_ok": True,
                    "bend_radius_violation_count": 0,
                    "bend_radius_violations": [],
                    "min_bend_radius_required": pipe.min_bend_radius,
                    "min_bend_radius_observed": float("inf"),
                    "bend_rule_violation_count": 0,
                    "bend_rule_violations": [],
                    "input_invalid": True,
                    "degraded_result": False,
                    "error": f"Invalid input endpoints: {', '.join(reasons)}",
                    "local_reroute_applied": False,
                    "local_reroute_reason": None,
                    "rerouted": False,
                    "reroute_reason": None,
                    "affected_by_changed_region": False,
                    "cbs_fallback_applied": False,
                    "cbs_fallback_reason": None,
                    "cbs_rerouted": False,
                    "cbs_constraints_count": 0,
                    "cbs_reroute_reason": None,
                }
            )
            continue

        for att in attempts:
            for clamp_candidates in clamp_candidate_sets:
                ares = astar_route(
                    grid=grid,
                    start_world=pipe.start,
                    end_world=pipe.end,
                    inflated_obstacles=inflated_obstacles,
                    dynamic_blocks=dynamic_blocks,
                    clamp_candidates=clamp_candidates,
                    clamp_reward_weight=att["clamp_reward_weight"],
                    turn_penalty=att["turn_penalty"],
                    bend_smoothness_penalty=att["bend_smoothness_penalty"],
                    bend_radius_penalty=att["bend_radius_penalty"],
                    length_weight=att["length_weight"],
                )
                if not ares.success:
                    last_error = ares.error
                    continue

                raw_path = ares.path[:]
                protected_points = _clamp_hit_points(raw_path, own_clamps)
                path_smoothed_protected = smooth_path(
                    grid=grid,
                    path=ares.path,
                    inflated_obstacles=inflated_obstacles,
                    dynamic_blocks=dynamic_blocks,
                    protected_points=protected_points,
                )
                path_smoothed_plain = smooth_path(
                    grid=grid,
                    path=ares.path,
                    inflated_obstacles=inflated_obstacles,
                    dynamic_blocks=dynamic_blocks,
                    protected_points=[],
                )

                variants = [
                    ("raw", raw_path),
                    ("smoothed_protected", path_smoothed_protected),
                    ("smoothed_plain", path_smoothed_plain),
                ]
                local_best_label = "raw"
                local_best_path = raw_path
                local_best_score = _evaluate_path_variant(raw_path, pipe, results, pipe_by_id, case.clamp_candidates)
                for label, cand in variants[1:]:
                    if not cand:
                        continue
                    if not _path_is_collision_free(grid, cand, inflated_obstacles, dynamic_blocks):
                        continue
                    score = _evaluate_path_variant(cand, pipe, results, pipe_by_id, case.clamp_candidates)
                    if score < local_best_score:
                        local_best_score = score
                        local_best_path = cand
                        local_best_label = label

                candidate_choice = {
                    "attempt_name": att["name"],
                    "raw_path": raw_path,
                    "smoothed_path": path_smoothed_protected,
                    "smoothed_plain_path": path_smoothed_plain,
                    "final_path": local_best_path,
                    "final_label": local_best_label,
                    "score": local_best_score,
                }
                if best_choice is None or candidate_choice["score"] < best_choice["score"]:
                    best_choice = candidate_choice

                # Early stop when bend violations are eliminated.
                if local_best_score[0] == 0:
                    break
            if best_choice is not None and best_choice["score"][0] == 0:
                break

        if best_choice is None:
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
                    "own_clamps": [c.id for c in own_clamps],
                    "missed_clamps": [c.id for c in own_clamps],
                    "min_bend_radius_ok": True,
                    "bend_radius_violation_count": 0,
                    "bend_radius_violations": [],
                    "min_bend_radius_required": pipe.min_bend_radius,
                    "min_bend_radius_observed": float("inf"),
                    "bend_rule_violation_count": 0,
                    "bend_rule_violations": [],
                    "input_invalid": False,
                    "degraded_result": False,
                    "error": last_error or "No path found.",
                    "local_reroute_applied": False,
                    "local_reroute_reason": None,
                    "rerouted": False,
                    "reroute_reason": None,
                    "affected_by_changed_region": False,
                    "cbs_fallback_applied": False,
                    "cbs_fallback_reason": None,
                    "cbs_rerouted": False,
                    "cbs_constraints_count": 0,
                    "cbs_reroute_reason": None,
                }
            )
            continue

        raw_path = best_choice["raw_path"]
        path_smoothed_protected = best_choice["smoothed_path"]
        path_smoothed_plain = best_choice["smoothed_plain_path"]
        best_path = best_choice["final_path"]
        best_label = best_choice["final_label"]
        smoothing_applied = best_label != "raw"
        smoothing_reverted = False
        smoothing_revert_reason = None
        final_path = best_path
        if best_label == "raw" and path_smoothed_protected != raw_path and path_smoothed_plain != raw_path:
            smoothing_reverted = True
            smoothing_revert_reason = "smoothing_not_better_under_bend_priority"

        result = {
            "id": pipe.id,
            "success": True,
            "raw_path": [list(p) for p in raw_path],
            "smoothed_path": [list(p) for p in path_smoothed_protected],
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
            "own_clamps": [c.id for c in own_clamps],
            "missed_clamps": [],
            "min_bend_radius_ok": True,
            "bend_radius_violation_count": 0,
            "bend_radius_violations": [],
            "min_bend_radius_required": pipe.min_bend_radius,
            "min_bend_radius_observed": float("inf"),
            "bend_rule_violation_count": 0,
            "bend_rule_violations": [],
            "input_invalid": False,
            "degraded_result": False,
            "error": None,
            "local_reroute_applied": False,
            "local_reroute_reason": None,
            "rerouted": False,
            "reroute_reason": None,
            "affected_by_changed_region": False,
            "cbs_fallback_applied": False,
            "cbs_fallback_reason": None,
            "cbs_rerouted": False,
            "cbs_constraints_count": 0,
            "cbs_reroute_reason": None,
        }
        _update_pipe_metrics(result, case.clamp_candidates, pipe)
        results.append(result)
        dynamic_blocks |= _rasterize_path_to_dynamic_blocks(
            grid,
            [tuple(p) for p in result["path"]],
            inflate_dist + grid.resolution * 0.5,
            protected_cells=terminal_allowance,
        )

    conflicts = _select_global_paths(results, pipe_by_id, case.clamp_candidates)
    conflicts = _apply_local_reroute(case, grid, results, pipe_by_id, conflicts)
    conflicts = apply_simplified_cbs_fallback(
        case=case,
        grid=grid,
        results=results,
        pipe_by_id=pipe_by_id,
        conflicts=conflicts,
        update_metrics=_update_pipe_metrics,
        max_iterations=8,
    )
    conflicts = _resolve_conflicts_by_priority(case, grid, results, pipe_by_id, conflicts)

    for item in results:
        item["conflicts"] = []
        item["conflict_count"] = 0

    for conflict in conflicts:
        for item in results:
            if item["id"] in (conflict["pipe_a"], conflict["pipe_b"]):
                item["conflicts"].append(conflict)

    for item in results:
        item["conflict_count"] = len(item["conflicts"])
        if item.get("success"):
            item["degraded_result"] = bool(
                int(item.get("bend_rule_violation_count", 0)) > 0
                or int(item.get("conflict_count", 0)) > 0
                or not bool(item.get("min_bend_radius_ok", True))
            )
        else:
            item["degraded_result"] = False
        item.setdefault("input_invalid", False)

    invalid_pipe_ids = sorted(set(original_issues.keys()) | set(unresolved_issues.keys()))
    invalid_reason = "; ".join(f"{pid}:{','.join(unresolved_issues.get(pid, original_issues.get(pid, [])))}" for pid in invalid_pipe_ids)
    return {
        "pipes": results,
        "conflicts": conflicts,
        "input_invalid": len(invalid_pipe_ids) > 0,
        "invalid_pipe_ids": invalid_pipe_ids,
        "invalid_reason": invalid_reason,
    }


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


def _global_quality_score(results: list[dict], conflicts: list[dict]) -> tuple[int, int, int, int, float]:
    failed_count = sum(1 for p in results if not p.get("success"))
    total_conflicts = len(conflicts)
    total_bend_violations = sum(int(p.get("bend_rule_violation_count", 0)) for p in results if p.get("success"))
    degraded_pipe_count = sum(
        1
        for p in results
        if p.get("success")
        and (
            int(p.get("bend_rule_violation_count", 0)) > 0
            or int(p.get("conflict_count", 0)) > 0
            or not bool(p.get("min_bend_radius_ok", True))
        )
    )
    total_length = sum(float(p.get("length", 0.0)) for p in results if p.get("success"))
    return (failed_count, total_conflicts, total_bend_violations, degraded_pipe_count, total_length)


def _resolve_conflicts_by_priority(
    case: RoutingCase,
    grid: Grid3D,
    results: list[dict],
    pipe_by_id: dict[str, Pipe],
    conflicts: list[dict],
) -> list[dict]:
    if not conflicts:
        return conflicts

    current_conflicts = conflicts
    max_rounds = 8
    for _ in range(max_rounds):
        if not current_conflicts:
            break

        involvement: dict[str, int] = {}
        for c in current_conflicts:
            involvement[c["pipe_a"]] = involvement.get(c["pipe_a"], 0) + 1
            involvement[c["pipe_b"]] = involvement.get(c["pipe_b"], 0) + 1
        candidates = sorted(involvement.keys(), key=lambda pid: involvement[pid], reverse=True)

        improved = False
        base_score = _global_quality_score(results, current_conflicts)
        for pid in candidates:
            item = next((x for x in results if x["id"] == pid and x.get("success")), None)
            if item is None:
                continue
            pipe = pipe_by_id[pid]
            ok, new_path, _err = reroute_single_pipe_locally(grid, case, pipe, results)
            if not ok or not new_path:
                continue

            old = {
                "raw_path": item.get("raw_path", []),
                "smoothed_path": item.get("smoothed_path", []),
                "path": item.get("path", []),
                "smoothing_applied": item.get("smoothing_applied", False),
                "smoothing_reverted": item.get("smoothing_reverted", False),
                "smoothing_revert_reason": item.get("smoothing_revert_reason"),
            }

            item["raw_path"] = [list(p) for p in new_path]
            item["smoothed_path"] = [list(p) for p in new_path]
            item["path"] = [list(p) for p in new_path]
            item["smoothing_applied"] = False
            item["smoothing_reverted"] = False
            item["smoothing_revert_reason"] = None
            _update_pipe_metrics(item, case.clamp_candidates, pipe)

            trial_conflicts = detect_pipe_conflicts(results, pipe_by_id)
            for r in results:
                r["conflict_count"] = sum(
                    1 for cc in trial_conflicts if r["id"] in (cc["pipe_a"], cc["pipe_b"])
                )
            trial_score = _global_quality_score(results, trial_conflicts)
            if trial_score < base_score:
                current_conflicts = trial_conflicts
                improved = True
                break

            item["raw_path"] = old["raw_path"]
            item["smoothed_path"] = old["smoothed_path"]
            item["path"] = old["path"]
            item["smoothing_applied"] = old["smoothing_applied"]
            item["smoothing_reverted"] = old["smoothing_reverted"]
            item["smoothing_revert_reason"] = old["smoothing_revert_reason"]
            _update_pipe_metrics(item, case.clamp_candidates, pipe)

        if not improved:
            break

    return current_conflicts
