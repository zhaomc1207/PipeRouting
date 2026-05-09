from pathlib import Path

from pipe_routing.cbs import make_conflict_constraint, score_solution, simplified_cbs
from pipe_routing.io import Pipe, RoutingCase, Workspace, load_routing_case
from pipe_routing.multi_pipe import route_pipes_sequentially
import pipe_routing.cbs as cbs


def test_make_conflict_constraint_from_point() -> None:
    conflict = {"pipe_a": "a", "pipe_b": "b", "conflict_point": [10.0, 20.0, 30.0]}
    box = make_conflict_constraint(conflict, padding=5.0)
    assert box["min"] == [5.0, 15.0, 25.0]
    assert box["max"] == [15.0, 25.0, 35.0]


def test_score_solution_prefers_fewer_conflicts() -> None:
    a = score_solution([{"success": True, "length": 10.0, "bend_rule_violation_count": 0, "path": [[0, 0, 0], [10, 0, 0]]}], [{}])
    b = score_solution([{"success": True, "length": 100.0, "bend_rule_violation_count": 10, "path": [[0, 0, 0], [10, 0, 0]]}], [])
    assert b < a


def test_score_solution_prefers_less_bend_and_turn_before_length() -> None:
    # Same conflicts, same failed pipes -> bend violations and turns should dominate length.
    worse_shape_short = score_solution(
        [{"success": True, "length": 10.0, "bend_rule_violation_count": 2, "path": [[0, 0, 0], [5, 0, 0], [5, 5, 0], [10, 5, 0]]}],
        [],
    )
    better_shape_long = score_solution(
        [{"success": True, "length": 50.0, "bend_rule_violation_count": 0, "path": [[0, 0, 0], [25, 0, 0], [50, 0, 0]]}],
        [],
    )
    assert better_shape_long < worse_shape_short


def test_score_solution_uses_length_last() -> None:
    # All earlier dimensions equal; shorter length should win only then.
    a = score_solution(
        [{"success": True, "length": 40.0, "bend_rule_violation_count": 0, "path": [[0, 0, 0], [20, 0, 0], [40, 0, 0]]}],
        [],
    )
    b = score_solution(
        [{"success": True, "length": 20.0, "bend_rule_violation_count": 0, "path": [[0, 0, 0], [10, 0, 0], [20, 0, 0]]}],
        [],
    )
    assert b < a


def test_simplified_cbs_no_conflict_keeps_results() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    initial = route_pipes_sequentially(case)
    out = simplified_cbs(case, initial, max_iterations=3, padding=40.0)
    assert out["cbs_enabled"] is True
    assert out["cbs_remaining_conflicts"] == 0
    assert out["cbs_iterations"] == 0


def test_simplified_cbs_attempts_on_conflict_case() -> None:
    case = RoutingCase(
        workspace=Workspace((0, 0, 0), (120, 120, 120), 2),
        obstacles=[],
        pipes=[
            Pipe("p1", (0, 0, 0), (100, 0, 0), 10, 5, 10),
            Pipe("p2", (0, 8, 0), (100, 8, 0), 10, 5, 10),
        ],
        clamp_candidates=[],
    )
    initial = {
        "pipes": [
            {
                "id": "p1",
                "success": True,
                "raw_path": [[0, 0, 0], [100, 0, 0]],
                "smoothed_path": [[0, 0, 0], [100, 0, 0]],
                "path": [[0, 0, 0], [100, 0, 0]],
                "length": 100.0,
                "bend_count": 0,
                "bend_rule_violation_count": 0,
            },
            {
                "id": "p2",
                "success": True,
                "raw_path": [[0, 8, 0], [100, 8, 0]],
                "smoothed_path": [[0, 8, 0], [100, 8, 0]],
                "path": [[0, 8, 0], [100, 8, 0]],
                "length": 100.0,
                "bend_count": 0,
                "bend_rule_violation_count": 0,
            },
        ],
        "conflicts": [],
    }
    out = simplified_cbs(case, initial, max_iterations=3, padding=15.0)
    assert out["cbs_enabled"] is True
    assert len(out["cbs_log"]) >= 1
    assert out["cbs_iterations"] >= 0
    # At least it should not get worse than starting obvious conflict state.
    assert out["cbs_remaining_conflicts"] <= 1


def test_simplified_cbs_can_reduce_conflict_with_candidate(monkeypatch) -> None:
    case = RoutingCase(
        workspace=Workspace((0, 0, 0), (120, 120, 120), 2),
        obstacles=[],
        pipes=[
            Pipe("p1", (0, 0, 0), (100, 0, 0), 10, 5, 10),
            Pipe("p2", (0, 8, 0), (100, 8, 0), 10, 5, 10),
        ],
        clamp_candidates=[],
    )
    initial = {
        "pipes": [
            {"id": "p1", "success": True, "raw_path": [[0, 0, 0], [100, 0, 0]], "smoothed_path": [[0, 0, 0], [100, 0, 0]], "path": [[0, 0, 0], [100, 0, 0]], "length": 100.0, "bend_count": 0, "bend_rule_violation_count": 0},
            {"id": "p2", "success": True, "raw_path": [[0, 8, 0], [100, 8, 0]], "smoothed_path": [[0, 8, 0], [100, 8, 0]], "path": [[0, 8, 0], [100, 8, 0]], "length": 100.0, "bend_count": 0, "bend_rule_violation_count": 0},
        ],
        "conflicts": [],
    }

    def _fake_reroute(_case, results, pipe_id, _constraint):
        out = [dict(x) for x in results]
        if pipe_id == "p2":
            p2 = next(x for x in out if x["id"] == "p2")
            p2["path"] = [[0, 40, 0], [100, 40, 0]]
            p2["raw_path"] = [[0, 40, 0], [100, 40, 0]]
            p2["smoothed_path"] = [[0, 40, 0], [100, 40, 0]]
            return out
        return None

    monkeypatch.setattr(cbs, "_reroute_one_pipe_with_constraint", _fake_reroute)
    out = simplified_cbs(case, initial, max_iterations=3, padding=15.0)
    assert out["cbs_resolved_conflicts"] >= 1
    assert out["cbs_remaining_conflicts"] == 0


def test_simplified_cbs_rejects_candidate_with_bend_violations(monkeypatch) -> None:
    case = RoutingCase(
        workspace=Workspace((0, 0, 0), (120, 120, 120), 2),
        obstacles=[],
        pipes=[
            Pipe("p1", (0, 0, 0), (100, 0, 0), 10, 5, 30),
            Pipe("p2", (0, 8, 0), (100, 8, 0), 10, 5, 30),
        ],
        clamp_candidates=[],
    )
    initial = {
        "pipes": [
            {"id": "p1", "success": True, "raw_path": [[0, 0, 0], [100, 0, 0]], "smoothed_path": [[0, 0, 0], [100, 0, 0]], "path": [[0, 0, 0], [100, 0, 0]], "length": 100.0, "bend_count": 0, "bend_rule_violation_count": 0, "min_bend_radius_ok": True},
            {"id": "p2", "success": True, "raw_path": [[0, 8, 0], [100, 8, 0]], "smoothed_path": [[0, 8, 0], [100, 8, 0]], "path": [[0, 8, 0], [100, 8, 0]], "length": 100.0, "bend_count": 0, "bend_rule_violation_count": 0, "min_bend_radius_ok": True},
        ],
        "conflicts": [],
    }

    def _bad_reroute(_case, results, pipe_id, _constraint):
        out = [dict(x) for x in results]
        if pipe_id == "p2":
            p2 = next(x for x in out if x["id"] == "p2")
            # Sharp zig-zag to trigger bend violations under min_bend_radius=30
            p2["path"] = [[0, 8, 0], [20, 8, 0], [20, 20, 0], [40, 20, 0], [40, 8, 0], [100, 8, 0]]
            p2["raw_path"] = p2["path"]
            p2["smoothed_path"] = p2["path"]
            p2["bend_rule_violation_count"] = 3
            p2["min_bend_radius_ok"] = False
            return out
        return None

    monkeypatch.setattr(cbs, "_reroute_one_pipe_with_constraint", _bad_reroute)
    out = simplified_cbs(case, initial, max_iterations=2, padding=15.0)
    # Candidate should be rejected; conflict remains unresolved.
    assert out["cbs_remaining_conflicts"] >= 1
