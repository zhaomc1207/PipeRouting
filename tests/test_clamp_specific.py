from pathlib import Path

from pipe_routing.cbs import simplified_cbs
from pipe_routing.io import load_routing_case
from pipe_routing.multi_pipe import route_pipes_sequentially


def test_clamp_applies_to_pipe_specific_metrics() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    initial = route_pipes_sequentially(case)
    result = simplified_cbs(case, initial, max_iterations=3, padding=40.0)
    by_id = {p["id"]: p for p in result["pipes"]}
    # clamp_p1_1 belongs only to pipe_1, should not appear in pipe_2 stats
    assert "clamp_p1_1" not in by_id["pipe_2"]["used_clamps"]
    assert "clamp_p1_1" not in by_id["pipe_2"]["min_distance_to_clamps"]


def test_pipe_uses_own_clamp_when_enter_radius() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    initial = route_pipes_sequentially(case)
    result = simplified_cbs(case, initial, max_iterations=3, padding=40.0)
    by_id = {p["id"]: p for p in result["pipes"]}
    # own clamp should be tracked in the per-pipe distance map
    assert "clamp_p1_1" in by_id["pipe_1"]["min_distance_to_clamps"]
    d = by_id["pipe_1"]["min_distance_to_clamps"]["clamp_p1_1"]
    radius = next(c.radius for c in case.clamp_candidates if c.id == "clamp_p1_1")
    if d <= radius:
        assert "clamp_p1_1" in by_id["pipe_1"]["used_clamps"]


def test_pipe2_not_affected_by_pipe1_clamp_and_hits_own() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    initial = route_pipes_sequentially(case)
    result = simplified_cbs(case, initial, max_iterations=3, padding=40.0)
    by_id = {p["id"]: p for p in result["pipes"]}
    assert "clamp_p1_1" not in by_id["pipe_2"]["min_distance_to_clamps"]
    assert "clamp_p2_1" in by_id["pipe_2"]["used_clamps"]
