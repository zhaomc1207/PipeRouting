from __future__ import annotations

from math import acos


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
    threshold = angle_threshold_deg * 3.1415926535 / 180.0
    for i in range(1, len(path) - 1):
        a = _sub(path[i], path[i - 1])
        b = _sub(path[i + 1], path[i])
        na, nb = _norm(a), _norm(b)
        if na == 0 or nb == 0:
            continue
        c = max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb)))
        ang = acos(c)
        if ang > threshold:
            count += 1
    return count
