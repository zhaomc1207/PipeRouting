from pathlib import Path

from pipe_routing.io import Pipe, RoutingCase, Workspace, load_routing_case
from pipe_routing.multi_pipe import route_pipes_sequentially
import pipe_routing.multi_pipe as multi_pipe


def test_multi_pipe_demo_runs() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    result = route_pipes_sequentially(case)
    assert "pipes" in result
    assert any(p["success"] for p in result["pipes"])


def test_multi_pipe_fields() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    result = route_pipes_sequentially(case)
    for p in result["pipes"]:
        assert "path" in p
        assert "raw_path" in p
        assert "smoothed_path" in p
        assert "length" in p
        assert "bend_count" in p
        assert "success" in p
        assert "smoothing_applied" in p
        assert "smoothing_reverted" in p
        assert "smoothing_revert_reason" in p
        assert "used_clamps" in p
        assert "min_distance_to_clamps" in p
        assert "own_clamps" in p
        assert "missed_clamps" in p
        assert "min_bend_radius_ok" in p
        assert "bend_radius_violation_count" in p
        assert "bend_radius_violations" in p
        assert "min_bend_radius_required" in p
        assert "min_bend_radius_observed" in p
        assert "bend_rule_violation_count" in p
        assert "bend_rule_violations" in p
        assert "local_reroute_applied" in p
        assert "local_reroute_reason" in p
        assert "rerouted" in p
        assert "reroute_reason" in p
        assert "affected_by_changed_region" in p
        assert "cbs_fallback_applied" in p
        assert "cbs_fallback_reason" in p
        assert "cbs_rerouted" in p
        assert "cbs_constraints_count" in p
        assert "cbs_reroute_reason" in p


def test_multi_pipe_clamp_metrics_keys() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    result = route_pipes_sequentially(case)
    by_id = {c.id: c for c in case.clamp_candidates}
    for p in result["pipes"]:
        if p["success"]:
            expected = {
                c.id
                for c in case.clamp_candidates
                if not c.applies_to or p["id"] in c.applies_to
            }
            assert set(p["min_distance_to_clamps"].keys()) == expected
            assert set(p["own_clamps"]) == expected
            assert set(p["missed_clamps"]).issubset(expected)
            assert all((cid in expected and p["id"] in (by_id[cid].applies_to or [p["id"]])) for cid in p["used_clamps"])


def test_multi_pipe_failure_does_not_crash() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    forced = case.pipes[0]
    blocked_case = type(case)(
        workspace=case.workspace,
        obstacles=case.obstacles,
        pipes=[type(forced)(forced.id, (2000, 2000, 2000), forced.end, forced.diameter, forced.clearance, forced.min_bend_radius)],
        clamp_candidates=case.clamp_candidates,
    )
    result = route_pipes_sequentially(blocked_case)
    assert result["pipes"][0]["success"] is False
    assert result["pipes"][0]["used_clamps"] == []
    assert result["pipes"][0]["min_distance_to_clamps"] == {}


def test_smoothing_reverts_when_empty(monkeypatch) -> None:
    case = load_routing_case(Path("data/demo_case.json"))

    def _empty(*args, **kwargs):
        return []

    monkeypatch.setattr(multi_pipe, "smooth_path", _empty)
    result = route_pipes_sequentially(case)
    first = next(p for p in result["pipes"] if p["success"])
    assert first["smoothing_reverted"] is True
    assert first["smoothing_revert_reason"] in (
        "smoothed_path_empty",
        "smoothing_not_better_under_bend_priority",
    )
    assert first["path"] == first["raw_path"]


def test_smoothing_reverts_when_conflict_increases(monkeypatch) -> None:
    ws = Workspace((0, 0, 0), (120, 120, 120), 2)
    p1 = Pipe("p1", (0, 0, 0), (100, 0, 0), 10, 5, 10)
    p2 = Pipe("p2", (0, 20, 0), (100, 20, 0), 10, 5, 10)
    case = RoutingCase(workspace=ws, obstacles=[], pipes=[p1, p2], clamp_candidates=[])

    original = multi_pipe.smooth_path

    def _force_conflict(grid, path, inflated_obstacles, dynamic_blocks=None, **kwargs):
        # Pull second pipe near first pipe to increase conflict while avoiding direct overlap.
        if path and path[0][1] == 20:
            return [(0, 6, 0), (100, 6, 0)]
        return original(grid, path, inflated_obstacles, dynamic_blocks)

    monkeypatch.setattr(multi_pipe, "smooth_path", _force_conflict)
    monkeypatch.setattr(multi_pipe, "_path_is_collision_free", lambda *args, **kwargs: True)
    result = route_pipes_sequentially(case)
    second = [p for p in result["pipes"] if p["id"] == "p2"][0]
    assert second["smoothing_reverted"] is True
    assert second["smoothing_revert_reason"] in (
        "smoothed_path_increases_conflicts",
        "smoothing_not_better_under_bend_priority",
    )
    assert second["path"] == second["raw_path"]


def test_smoothing_applied_successfully() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    result = route_pipes_sequentially(case)
    assert any(p["smoothing_applied"] and not p["smoothing_reverted"] for p in result["pipes"] if p["success"])


def test_global_conflict_recheck_reverts_smoothed_path(monkeypatch) -> None:
    ws = Workspace((0, 0, 0), (120, 120, 120), 2)
    p1 = Pipe("p1", (0, 0, 0), (100, 0, 0), 10, 5, 10)
    p2 = Pipe("p2", (0, 20, 0), (100, 20, 0), 10, 5, 10)
    case = RoutingCase(workspace=ws, obstacles=[], pipes=[p1, p2], clamp_candidates=[])

    monkeypatch.setattr(multi_pipe, "_count_conflicts_with_previous", lambda *args, **kwargs: 0)

    original = multi_pipe.smooth_path

    def _force_conflict(grid, path, inflated_obstacles, dynamic_blocks=None, **kwargs):
        if path and path[0][1] == 20:
            return [(0, 6, 0), (100, 6, 0)]
        return original(grid, path, inflated_obstacles, dynamic_blocks)

    monkeypatch.setattr(multi_pipe, "smooth_path", _force_conflict)
    monkeypatch.setattr(multi_pipe, "_path_is_collision_free", lambda *args, **kwargs: True)
    result = route_pipes_sequentially(case)
    second = [p for p in result["pipes"] if p["id"] == "p2"][0]
    assert second["smoothing_reverted"] is True
    assert second["smoothing_revert_reason"] in (
        "final_global_conflict_recheck",
        "smoothing_not_better_under_bend_priority",
    )
    assert second["path"] == second["raw_path"]


def test_demo_case_total_conflicts_zero() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    result = route_pipes_sequentially(case)
    assert len(result["conflicts"]) == 0
    assert all(p["conflict_count"] == 0 for p in result["pipes"] if p["success"])


def test_demo_case_has_pipe_specific_clamp_usage() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    result = route_pipes_sequentially(case)
    assert any(p["used_clamps"] for p in result["pipes"] if p["success"])


def test_demo_case_bend_violations_zero() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    result = route_pipes_sequentially(case)
    assert all(p["bend_rule_violation_count"] == 0 for p in result["pipes"] if p["success"])


def test_local_reroute_not_applied_when_no_conflicts() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    result = route_pipes_sequentially(case)
    assert all(p["local_reroute_applied"] is False for p in result["pipes"] if p["success"])


def test_cbs_fallback_not_applied_when_no_conflicts() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    result = route_pipes_sequentially(case)
    assert all(p["cbs_fallback_applied"] is False for p in result["pipes"] if p["success"])
