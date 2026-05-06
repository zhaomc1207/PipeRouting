from pathlib import Path

from pipe_routing.io import load_routing_case
from pipe_routing.multi_pipe import route_pipes_sequentially


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
        assert "length" in p
        assert "bend_count" in p
        assert "success" in p


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
