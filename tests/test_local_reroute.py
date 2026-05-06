from pathlib import Path

from pipe_routing.io import load_routing_case
from pipe_routing.local_reroute import find_affected_pipes, local_reroute, path_intersects_box
from pipe_routing.multi_pipe import route_pipes_sequentially


def test_path_intersects_box_true() -> None:
    path = [[0, 0, 0], [10, 10, 10], [20, 20, 20]]
    box = {"type": "box", "min": [5, 5, 5], "max": [8, 8, 8]}
    assert path_intersects_box(path, box)


def test_path_intersects_box_false() -> None:
    path = [[0, 0, 0], [1, 1, 1], [2, 2, 2]]
    box = {"type": "box", "min": [10, 10, 10], "max": [12, 12, 12]}
    assert not path_intersects_box(path, box)


def test_find_affected_pipes_detects_intersection() -> None:
    previous_results = {
        "pipes": [
            {"id": "pipe_1", "success": True, "path": [[0, 0, 0], [10, 0, 0]]},
            {"id": "pipe_2", "success": True, "path": [[0, 20, 0], [10, 20, 0]]},
        ],
        "conflicts": [],
    }
    changed_region = {"id": "r1", "type": "box", "min": [4, -1, -1], "max": [6, 1, 1]}
    affected = find_affected_pipes(previous_results, changed_region)
    assert affected == ["pipe_1"]


def test_local_reroute_keeps_unaffected_pipe_path() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    prev = route_pipes_sequentially(case)
    # tiny region around pipe_1 start to affect it but not all pipes
    changed = {"id": "changed", "type": "box", "min": [50, 110, 110], "max": [90, 140, 140]}
    new_res = local_reroute(case, prev, changed)

    prev_map = {p["id"]: p for p in prev["pipes"]}
    new_map = {p["id"]: p for p in new_res["pipes"]}
    for pid, p in new_map.items():
        if not p["affected_by_changed_region"] and p["success"] and prev_map[pid]["success"]:
            assert p["path"] == prev_map[pid]["path"]


def test_local_reroute_marks_affected_pipe() -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    prev = route_pipes_sequentially(case)
    changed = {"id": "changed", "type": "box", "min": [50, 110, 110], "max": [90, 140, 140]}
    new_res = local_reroute(case, prev, changed)
    assert any(p["affected_by_changed_region"] for p in new_res["pipes"])
    assert any(p["rerouted"] for p in new_res["pipes"] if p["affected_by_changed_region"])
