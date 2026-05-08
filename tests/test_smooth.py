from pipe_routing.grid import Grid3D, inflate_obstacle
from pipe_routing.io import Obstacle, Workspace
from pipe_routing.smooth import smooth_path


def test_smooth_reduces_points_in_open_space() -> None:
    grid = Grid3D(Workspace((0, 0, 0), (100, 100, 100), 10))
    path = [(0, 0, 0), (10, 0, 0), (20, 0, 0), (30, 0, 0)]
    smoothed = smooth_path(grid, path, [])
    assert smoothed[0] == path[0]
    assert smoothed[-1] == path[-1]
    assert len(smoothed) <= len(path)


def test_smooth_keeps_turn_when_obstacle_blocks_shortcut() -> None:
    grid = Grid3D(Workspace((0, 0, 0), (100, 100, 100), 10))
    obstacle = Obstacle("obs", "box", (10, 0, 0), (20, 20, 20))
    inflated = [inflate_obstacle(obstacle, 0)]
    path = [(0, 0, 0), (0, 30, 0), (30, 30, 0)]
    smoothed = smooth_path(grid, path, inflated)
    assert smoothed[0] == path[0]
    assert smoothed[-1] == path[-1]
    assert len(smoothed) >= 3


def test_smooth_keeps_protected_point() -> None:
    grid = Grid3D(Workspace((0, 0, 0), (100, 100, 100), 10))
    path = [(0, 0, 0), (10, 0, 0), (20, 0, 0), (30, 0, 0)]
    smoothed = smooth_path(grid, path, [], protected_points=[(20, 0, 0)])
    assert (20, 0, 0) in smoothed


def test_smooth_protected_still_smooths_other_points() -> None:
    grid = Grid3D(Workspace((0, 0, 0), (200, 200, 200), 10))
    path = [(0, 0, 0), (10, 0, 0), (20, 0, 0), (30, 0, 0), (40, 0, 0), (50, 0, 0)]
    smoothed = smooth_path(grid, path, [], protected_points=[(20, 0, 0)])
    assert (20, 0, 0) in smoothed
    assert len(smoothed) < len(path)
