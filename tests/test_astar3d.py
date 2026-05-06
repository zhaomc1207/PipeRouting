from pipe_routing.astar3d import astar_route
from pipe_routing.grid import Grid3D, inflate_obstacle
from pipe_routing.io import Obstacle, Workspace


def test_astar_reaches_goal() -> None:
    grid = Grid3D(Workspace((0, 0, 0), (100, 100, 100), 10))
    result = astar_route(grid, (0, 0, 0), (100, 100, 100), [])
    assert result.success
    assert result.path[0] == (0, 0, 0)
    assert result.path[-1] == (100, 100, 100)


def test_astar_avoids_box_obstacle() -> None:
    grid = Grid3D(Workspace((0, 0, 0), (100, 100, 100), 10))
    obs = Obstacle("o1", "box", (40, 40, 40), (60, 60, 60))
    result = astar_route(grid, (0, 0, 0), (100, 100, 100), [inflate_obstacle(obs, 0)])
    assert result.success
    assert all(not (40 <= p[0] <= 60 and 40 <= p[1] <= 60 and 40 <= p[2] <= 60) for p in result.path)


def test_astar_no_path_returns_failure() -> None:
    grid = Grid3D(Workspace((0, 0, 0), (40, 40, 40), 10))
    obs = Obstacle("wall", "box", (0, 10, 0), (40, 30, 40))
    result = astar_route(grid, (0, 0, 0), (40, 40, 40), [obs])
    assert not result.success
    assert result.error is not None
