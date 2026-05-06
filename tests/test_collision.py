from pipe_routing.collision import detect_pipe_conflicts, segment_segment_distance
from pipe_routing.io import Pipe


def test_segment_distance_reasonable() -> None:
    d = segment_segment_distance((0, 0, 0), (10, 0, 0), (0, 10, 0), (10, 10, 0))
    assert 9.0 <= d <= 11.0


def test_no_conflict_when_far() -> None:
    routed = [
        {"id": "a", "success": True, "path": [[0, 0, 0], [100, 0, 0]]},
        {"id": "b", "success": True, "path": [[0, 100, 0], [100, 100, 0]]},
    ]
    pipes = {
        "a": Pipe("a", (0, 0, 0), (100, 0, 0), 10, 5, 10),
        "b": Pipe("b", (0, 100, 0), (100, 100, 0), 10, 5, 10),
    }
    assert detect_pipe_conflicts(routed, pipes) == []


def test_conflict_when_close() -> None:
    routed = [
        {"id": "a", "success": True, "path": [[0, 0, 0], [100, 0, 0]]},
        {"id": "b", "success": True, "path": [[0, 8, 0], [100, 8, 0]]},
    ]
    pipes = {
        "a": Pipe("a", (0, 0, 0), (100, 0, 0), 10, 5, 10),
        "b": Pipe("b", (0, 8, 0), (100, 8, 0), 10, 5, 10),
    }
    conflicts = detect_pipe_conflicts(routed, pipes)
    assert len(conflicts) >= 1
