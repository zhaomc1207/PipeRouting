from math import inf

from pipe_routing.pipe_rules import (
    check_min_bend_radius,
    compute_turn_angle,
    estimate_bend_radius,
    summarize_bend_rules,
)


def test_estimate_bend_radius_collinear_returns_inf() -> None:
    r = estimate_bend_radius((0, 0, 0), (10, 0, 0), (20, 0, 0))
    assert r == inf


def test_sharp_turn_triggers_bend_violation() -> None:
    path = [(0, 0, 0), (10, 0, 0), (10, 10, 0)]
    res = check_min_bend_radius(path, min_bend_radius=20.0)
    assert res["bend_rule_violation_count"] >= 1
    assert res["min_bend_radius_required"] == 20.0


def test_gentle_path_no_bend_violation() -> None:
    path = [(0, 0, 0), (100, 0, 0), (200, 1, 0)]
    res = summarize_bend_rules(path, min_bend_radius=30.0)
    assert res["bend_rule_violation_count"] == 0


def test_compute_turn_angle_reasonable() -> None:
    a = compute_turn_angle((0, 0, 0), (10, 0, 0), (10, 10, 0))
    assert 80.0 <= a <= 100.0
