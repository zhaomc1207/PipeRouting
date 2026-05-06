from __future__ import annotations

from math import acos, inf, sqrt


Vec3 = tuple[float, float, float]


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm(v: Vec3) -> float:
    return (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5


def path_length(path: list[Vec3]) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path) - 1):
        total += _norm(_sub(path[i + 1], path[i]))
    return total


def bend_count(path: list[Vec3], angle_threshold_deg: float = 10.0) -> int:
    if len(path) < 3:
        return 0
    count = 0
    for i in range(1, len(path) - 1):
        ang = compute_turn_angle(path[i - 1], path[i], path[i + 1])
        if ang > angle_threshold_deg:
            count += 1
    return count


def compute_turn_angle(p_prev: Vec3, p_curr: Vec3, p_next: Vec3) -> float:
    """Return turn angle at p_curr in degrees, range [0, 180]."""
    a = _sub(p_prev, p_curr)
    b = _sub(p_next, p_curr)
    na, nb = _norm(a), _norm(b)
    if na == 0 or nb == 0:
        return 0.0
    c = max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb)))
    return acos(c) * 180.0 / 3.1415926535


def _triangle_area2(a: Vec3, b: Vec3, c: Vec3) -> float:
    ab = _sub(b, a)
    ac = _sub(c, a)
    cx = ab[1] * ac[2] - ab[2] * ac[1]
    cy = ab[2] * ac[0] - ab[0] * ac[2]
    cz = ab[0] * ac[1] - ab[1] * ac[0]
    return sqrt(cx * cx + cy * cy + cz * cz)


def estimate_bend_radius(p_prev: Vec3, p_curr: Vec3, p_next: Vec3) -> float:
    """Estimate bend radius by circumcircle radius through 3 points."""
    a = _norm(_sub(p_curr, p_prev))
    b = _norm(_sub(p_next, p_curr))
    c = _norm(_sub(p_next, p_prev))
    if a == 0 or b == 0 or c == 0:
        return inf
    area2 = _triangle_area2(p_prev, p_curr, p_next)
    if area2 <= 1e-8:
        return inf
    area = area2 * 0.5
    return (a * b * c) / (4.0 * area)


def check_min_bend_radius(path: list[Vec3], min_bend_radius: float) -> dict:
    violations: list[dict] = []
    observed = inf
    required = float(min_bend_radius)

    if len(path) < 3 or min_bend_radius <= 0:
        return {
            "min_bend_radius_required": required,
            "min_bend_radius_observed": observed,
            "bend_rule_violation_count": 0,
            "bend_rule_violations": violations,
            # Backward-compatible aliases
            "min_bend_radius_ok": True,
            "bend_radius_violation_count": 0,
            "bend_radius_violations": violations,
        }

    for i in range(1, len(path) - 1):
        p_prev, p_curr, p_next = path[i - 1], path[i], path[i + 1]
        radius = estimate_bend_radius(p_prev, p_curr, p_next)
        angle_deg = compute_turn_angle(p_prev, p_curr, p_next)
        if radius != inf:
            observed = min(observed, radius)
        if radius < min_bend_radius:
            violations.append(
                {
                    "point_index": i,
                    "point": [p_curr[0], p_curr[1], p_curr[2]],
                    "estimated_radius": radius,
                    "required_radius": required,
                    "turn_angle_degrees": angle_deg,
                }
            )

    return {
        "min_bend_radius_required": required,
        "min_bend_radius_observed": observed,
        "bend_rule_violation_count": len(violations),
        "bend_rule_violations": violations,
        # Backward-compatible aliases
        "min_bend_radius_ok": len(violations) == 0,
        "bend_radius_violation_count": len(violations),
        "bend_radius_violations": violations,
    }


def summarize_bend_rules(path: list[Vec3], min_bend_radius: float) -> dict:
    return check_min_bend_radius(path, min_bend_radius)
