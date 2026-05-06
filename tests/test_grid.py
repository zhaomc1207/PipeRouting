from pipe_routing.grid import Grid3D, inflate_obstacle, is_cell_in_box
from pipe_routing.io import Obstacle, Workspace


def test_world_index_roundtrip() -> None:
    grid = Grid3D(Workspace((0, 0, 0), (100, 100, 100), 10))
    idx = grid.world_to_index((20, 30, 40))
    assert idx == (2, 3, 4)
    assert grid.index_to_world(idx) == (20, 30, 40)


def test_inflate_obstacle() -> None:
    obs = Obstacle("o1", "box", (10, 10, 10), (20, 20, 20))
    inf = inflate_obstacle(obs, 5)
    assert inf.min_corner == (5, 5, 5)
    assert inf.max_corner == (25, 25, 25)


def test_cell_occupied_and_bounds() -> None:
    grid = Grid3D(Workspace((0, 0, 0), (100, 100, 100), 10))
    obs = Obstacle("o1", "box", (20, 20, 20), (40, 40, 40))
    assert is_cell_in_box(grid, (2, 2, 2), obs)
    assert not is_cell_in_box(grid, (8, 8, 8), obs)
    assert grid.in_bounds((0, 0, 0))
    assert not grid.in_bounds((-1, 0, 0))
