from __future__ import annotations

from math import sqrt

from .io import Pipe

Vec3 = tuple[float, float, float]


def point_distance(a: Vec3, b: Vec3) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def segment_segment_distance(p1: Vec3, p2: Vec3, q1: Vec3, q2: Vec3) -> float:
    """Minimum distance between 3D segments using endpoint sampling fallback."""
    # For MVP robustness and low dependency, use discretized projection checks.
    samples = 20
    min_dist = float("inf")
    for i in range(samples + 1):
        t = i / samples
        p = (p1[0] + (p2[0] - p1[0]) * t, p1[1] + (p2[1] - p1[1]) * t, p1[2] + (p2[2] - p1[2]) * t)
        for j in range(samples + 1):
            u = j / samples
            q = (q1[0] + (q2[0] - q1[0]) * u, q1[1] + (q2[1] - q1[1]) * u, q1[2] + (q2[2] - q1[2]) * u)
            min_dist = min(min_dist, point_distance(p, q))
    return min_dist


def detect_pipe_conflicts(
    routed: list[dict],
    pipe_by_id: dict[str, Pipe],
    distance_tolerance: float = 5.0,
) -> list[dict]:
    conflicts: list[dict] = []
    for i in range(len(routed)):
        for j in range(i + 1, len(routed)):
            a = routed[i]
            b = routed[j]
            if not a["success"] or not b["success"]:
                continue
            path_a = a["path"]
            path_b = b["path"]
            pipe_a = pipe_by_id[a["id"]]
            pipe_b = pipe_by_id[b["id"]]
            required = pipe_a.diameter / 2 + pipe_b.diameter / 2 + max(pipe_a.clearance, pipe_b.clearance)
            for sa in range(len(path_a) - 1):
                p1, p2 = tuple(path_a[sa]), tuple(path_a[sa + 1])
                for sb in range(len(path_b) - 1):
                    q1, q2 = tuple(path_b[sb]), tuple(path_b[sb + 1])
                    d = segment_segment_distance(p1, p2, q1, q2)
                    if d + distance_tolerance < required:
                        conflicts.append(
                            {
                                "pipe_a": a["id"],
                                "pipe_b": b["id"],
                                "segment_a_index": sa,
                                "segment_b_index": sb,
                                "distance": d,
                                "required_distance": required,
                            }
                        )
    return conflicts
