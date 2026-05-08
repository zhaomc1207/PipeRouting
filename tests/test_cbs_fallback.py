from pipe_routing.cbs_fallback import apply_simplified_cbs_fallback
from pipe_routing.collision import detect_pipe_conflicts
from pipe_routing.grid import Grid3D
from pipe_routing.io import Pipe, RoutingCase, Workspace
import pipe_routing.cbs_fallback as cbs_fallback


def test_cbs_fallback_reduces_conflicts(monkeypatch) -> None:
    case = RoutingCase(
        workspace=Workspace((0, 0, 0), (100, 100, 100), 10),
        obstacles=[],
        pipes=[
            Pipe("a", (0, 0, 0), (100, 0, 0), 10, 5, 10),
            Pipe("b", (0, 8, 0), (100, 8, 0), 10, 5, 10),
        ],
        clamp_candidates=[],
    )
    pipe_by_id = {p.id: p for p in case.pipes}
    grid = Grid3D(case.workspace)
    results = [
        {"id": "a", "success": True, "path": [[0, 0, 0], [100, 0, 0]], "raw_path": [[0, 0, 0], [100, 0, 0]], "smoothed_path": [[0, 0, 0], [100, 0, 0]]},
        {"id": "b", "success": True, "path": [[0, 8, 0], [100, 8, 0]], "raw_path": [[0, 8, 0], [100, 8, 0]], "smoothed_path": [[0, 8, 0], [100, 8, 0]]},
    ]
    conflicts = detect_pipe_conflicts(results, pipe_by_id)
    assert len(conflicts) > 0

    def _fake_reroute(_grid, _case, pipe, _results):
        if pipe.id == "b":
            return True, [(0, 40, 0), (100, 40, 0)], None
        return False, [], "skip"

    monkeypatch.setattr(cbs_fallback, "reroute_single_pipe_locally", _fake_reroute)

    def _update_metrics(*args, **kwargs):
        return None

    out_conflicts = apply_simplified_cbs_fallback(case, grid, results, pipe_by_id, conflicts, _update_metrics, max_iterations=2)
    assert len(out_conflicts) == 0
    b = [p for p in results if p["id"] == "b"][0]
    assert b["cbs_fallback_applied"] is True

