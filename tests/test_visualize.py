from pathlib import Path

from pipe_routing.io import load_routing_case
from pipe_routing.multi_pipe import route_pipes_sequentially
from pipe_routing.visualize import create_tube_mesh, rounded_path_for_visualization, write_result_html
import run_demo


def test_create_tube_mesh_two_points_non_empty() -> None:
    mesh = create_tube_mesh([(0, 0, 0), (10, 0, 0)], radius=2.0, color="#ff0000", name="p1", segments=12)
    assert mesh is not None
    assert len(mesh.x) > 0
    assert len(mesh.i) > 0
    assert len(mesh.i) == len(mesh.j) == len(mesh.k)


def test_create_tube_mesh_empty_or_single_point() -> None:
    assert create_tube_mesh([], radius=2.0, color="#ff0000", name="p1") is None
    assert create_tube_mesh([(0, 0, 0)], radius=2.0, color="#ff0000", name="p1") is None


def test_rounded_path_for_visualization_preserves_endpoints() -> None:
    path = [(0, 0, 0), (10, 0, 0), (10, 10, 0)]
    rounded = rounded_path_for_visualization(path, elbow_radius=2.0, samples_per_corner=5)
    assert rounded[0] == path[0]
    assert rounded[-1] == path[-1]
    assert len(rounded) > len(path)


def test_create_tube_mesh_with_duplicate_points() -> None:
    path = [(0, 0, 0), (0, 0, 0), (10, 0, 0), (10, 0, 0), (10, 10, 0)]
    mesh = create_tube_mesh(path, radius=1.0, color="#00ff00", name="dup", segments=8)
    assert mesh is not None
    assert len(mesh.x) > 0
    assert len(mesh.i) > 0


def test_run_demo_generates_result_html() -> None:
    run_demo.main()
    html_path = Path("outputs/result.html")
    assert html_path.exists()
    assert html_path.stat().st_size > 0


def test_write_result_html_default_disables_rounded_elbows(tmp_path) -> None:
    case = load_routing_case(Path("data/demo_case.json"))
    result = route_pipes_sequentially(case)
    out = tmp_path / "result.html"
    write_result_html(case, result, out)
    assert out.exists()
    assert out.stat().st_size > 0
