from __future__ import annotations

from typing import Callable

from .collision import detect_pipe_conflicts
from .grid import Grid3D
from .io import Pipe, RoutingCase
from .local_reroute import reroute_single_pipe_locally


def apply_simplified_cbs_fallback(
    case: RoutingCase,
    grid: Grid3D,
    results: list[dict],
    pipe_by_id: dict[str, Pipe],
    conflicts: list[dict],
    update_metrics: Callable[[dict, list, Pipe], None],
    max_iterations: int = 3,
) -> list[dict]:
    """Conflict-driven fallback reroute loop (MVP CBS-like).

    Strategy:
    - Pick one conflicting pair each round.
    - Try rerouting each member while other pipes are treated as dynamic obstacles.
    - Accept only strict global conflict reduction.
    """
    if not conflicts:
        return conflicts

    current_conflicts = conflicts
    for _ in range(max_iterations):
        if not current_conflicts:
            break

        first = current_conflicts[0]
        candidates = [first["pipe_b"], first["pipe_a"]]
        improved = False

        for pid in candidates:
            item = next((x for x in results if x["id"] == pid and x.get("success")), None)
            if item is None:
                continue
            pipe = pipe_by_id[pid]
            ok, new_path, err = reroute_single_pipe_locally(grid, case, pipe, results)
            if not ok or not new_path:
                item["cbs_fallback_reason"] = err or "reroute_failed"
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
            update_metrics(item, case.clamp_candidates, pipe)

            trial_conflicts = detect_pipe_conflicts(results, pipe_by_id)
            if len(trial_conflicts) < len(current_conflicts):
                item["cbs_fallback_applied"] = True
                item["cbs_fallback_reason"] = "conflict_reduction"
                current_conflicts = trial_conflicts
                improved = True
                break

            item["raw_path"] = old["raw_path"]
            item["smoothed_path"] = old["smoothed_path"]
            item["path"] = old["path"]
            item["smoothing_applied"] = old["smoothing_applied"]
            item["smoothing_reverted"] = old["smoothing_reverted"]
            item["smoothing_revert_reason"] = old["smoothing_revert_reason"]
            item["cbs_fallback_applied"] = False
            item["cbs_fallback_reason"] = "no_improvement"
            update_metrics(item, case.clamp_candidates, pipe)

        if not improved:
            break

    return current_conflicts

