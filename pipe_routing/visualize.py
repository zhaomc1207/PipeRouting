from __future__ import annotations

from math import acos, cos, pi, sin, sqrt
from pathlib import Path

import plotly.graph_objects as go

from .io import RoutingCase


EPS = 1e-6


def _box_mesh(min_c: tuple[float, float, float], max_c: tuple[float, float, float]) -> go.Mesh3d:
    x0, y0, z0 = min_c
    x1, y1, z1 = max_c
    vertices = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]
    i = [0, 0, 0, 1, 1, 2, 4, 4, 5, 6, 7, 7]
    j = [1, 2, 3, 2, 5, 3, 5, 7, 6, 7, 4, 3]
    k = [2, 3, 1, 5, 2, 6, 7, 5, 2, 3, 0, 6]
    x, y, z = zip(*vertices)
    return go.Mesh3d(x=x, y=y, z=z, i=i, j=j, k=k, color="gray", opacity=0.25, showscale=False)


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if n <= EPS:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _norm(v: tuple[float, float, float]) -> float:
    return sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def _dedupe_path(path: list[list[float]] | list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    pts = [tuple(float(c) for c in p) for p in path]
    if not pts:
        return []
    out = [pts[0]]
    for p in pts[1:]:
        if _norm(_sub(p, out[-1])) >= EPS:
            out.append(p)
    return out


def rounded_path_for_visualization(
    path: list[list[float]] | list[tuple[float, float, float]],
    elbow_radius: float,
    samples_per_corner: int = 5,
    max_cut_ratio: float = 0.25,
) -> list[tuple[float, float, float]]:
    """Display-only rounded path for tube rendering; does not change algorithm output path."""
    pts = _dedupe_path(path)
    if len(pts) < 3 or elbow_radius <= 0:
        return pts

    out: list[tuple[float, float, float]] = [pts[0]]

    for i in range(1, len(pts) - 1):
        p_prev, p_curr, p_next = pts[i - 1], pts[i], pts[i + 1]
        v_in = _sub(p_prev, p_curr)
        v_out = _sub(p_next, p_curr)
        l_in = _norm(v_in)
        l_out = _norm(v_out)
        if l_in <= EPS or l_out <= EPS:
            out.append(p_curr)
            continue

        u_in = _normalize(v_in)
        u_out = _normalize(v_out)
        cos_theta = max(-1.0, min(1.0, _dot(u_in, u_out)))
        theta = acos(cos_theta)
        if theta <= 1e-3 or abs(theta - pi) <= 1e-3:
            out.append(p_curr)
            continue

        cut = min(elbow_radius, max_cut_ratio * l_in, max_cut_ratio * l_out)
        p_start = _add(p_curr, _scale(u_in, cut))
        p_end = _add(p_curr, _scale(u_out, cut))

        out.append(p_start)
        samples = max(3, samples_per_corner)
        for s in range(1, samples - 1):
            t = s / (samples - 1)
            blend = _normalize(_add(_scale(u_in, 1.0 - t), _scale(u_out, t)))
            out.append(_add(p_curr, _scale(blend, cut)))
        out.append(p_end)

    out.append(pts[-1])
    return _dedupe_path(out)


def _path_tangent(points: list[tuple[float, float, float]], idx: int) -> tuple[float, float, float]:
    if idx == 0:
        return _normalize(_sub(points[1], points[0]))
    if idx == len(points) - 1:
        return _normalize(_sub(points[-1], points[-2]))
    a = _normalize(_sub(points[idx], points[idx - 1]))
    b = _normalize(_sub(points[idx + 1], points[idx]))
    t = _normalize(_add(a, b))
    if _norm(t) <= EPS:
        t = b
    return t


def create_tube_mesh(
    path: list[list[float]] | list[tuple[float, float, float]],
    radius: float,
    color: str,
    name: str,
    segments: int = 12,
) -> go.Mesh3d | None:
    """Create continuous tube mesh by rings along whole path (not per-segment isolated cylinders)."""
    points = _dedupe_path(path)
    if radius <= 0 or len(points) < 2 or segments < 3:
        return None

    tangents = [_path_tangent(points, i) for i in range(len(points))]

    # Initial frame
    t0 = tangents[0]
    ref = (0.0, 0.0, 1.0) if abs(t0[2]) < 0.9 else (0.0, 1.0, 0.0)
    n = _normalize(_cross(t0, ref))
    if _norm(n) <= EPS:
        ref = (1.0, 0.0, 0.0)
        n = _normalize(_cross(t0, ref))
    b = _normalize(_cross(t0, n))

    ring_normals: list[tuple[float, float, float]] = [n]
    ring_binormals: list[tuple[float, float, float]] = [b]

    for i in range(1, len(points)):
        t = tangents[i]
        prev_n = ring_normals[-1]
        # Parallel transport-like re-orthogonalization to reduce flips/twists.
        n_proj = _sub(prev_n, _scale(t, _dot(prev_n, t)))
        n_new = _normalize(n_proj)
        if _norm(n_new) <= EPS:
            ref = (0.0, 0.0, 1.0) if abs(t[2]) < 0.9 else (0.0, 1.0, 0.0)
            n_new = _normalize(_cross(t, ref))
            if _norm(n_new) <= EPS:
                n_new = (1.0, 0.0, 0.0)
        b_new = _normalize(_cross(t, n_new))
        ring_normals.append(n_new)
        ring_binormals.append(b_new)

    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    ii: list[int] = []
    jj: list[int] = []
    kk: list[int] = []

    for idx, center in enumerate(points):
        n = ring_normals[idx]
        b = ring_binormals[idx]
        for s in range(segments):
            ang = 2.0 * pi * s / segments
            offset = _add(_scale(n, cos(ang) * radius), _scale(b, sin(ang) * radius))
            p = _add(center, offset)
            xs.append(p[0])
            ys.append(p[1])
            zs.append(p[2])

    for ring in range(len(points) - 1):
        a = ring * segments
        c = (ring + 1) * segments
        for s in range(segments):
            n = (s + 1) % segments
            a0 = a + s
            a1 = a + n
            c0 = c + s
            c1 = c + n
            ii.extend([a0, a0])
            jj.extend([c0, c1])
            kk.extend([c1, a1])

    return go.Mesh3d(
        x=xs,
        y=ys,
        z=zs,
        i=ii,
        j=jj,
        k=kk,
        color=color,
        opacity=0.85,
        name=name,
        hovertemplate=f"id={name}<extra></extra>",
        showscale=False,
    )


def write_result_html(
    case: RoutingCase,
    routing_result: dict,
    output_path: Path,
    enable_rounded_elbows: bool = False,
) -> None:
    fig = go.Figure()
    successful_pipe_count = 0

    for obs in case.obstacles:
        fig.add_trace(_box_mesh(obs.min_corner, obs.max_corner))

    colors = ["#1f77b4", "#2ca02c", "#ff7f0e", "#d62728", "#17becf", "#8c564b"]
    pipe_specs = {p.id: p for p in case.pipes}

    for idx, p in enumerate(routing_result["pipes"]):
        if not p["success"] or not p["path"]:
            continue
        successful_pipe_count += 1
        pts = p["path"]
        xs = [a[0] for a in pts]
        ys = [a[1] for a in pts]
        zs = [a[2] for a in pts]
        color = colors[idx % len(colors)]
        pipe_name = p["id"]
        if p.get("smoothing_reverted"):
            pipe_name = f"{pipe_name} (reverted)"

        spec = pipe_specs.get(p["id"])
        radius = (spec.diameter / 2.0) if spec is not None else 5.0
        vis_path = (
            rounded_path_for_visualization(
                pts,
                elbow_radius=max(radius * 1.2, 1.0),
                samples_per_corner=5,
                max_cut_ratio=0.25,
            )
            if enable_rounded_elbows
            else _dedupe_path(pts)
        )
        tube = create_tube_mesh(vis_path, radius=radius, color=color, name=pipe_name, segments=12)
        if tube is not None:
            tube.hovertemplate = (
                f"id={p['id']}<br>"
                f"smoothing_reverted={p.get('smoothing_reverted', False)}<br>"
                f"revert_reason={p.get('smoothing_revert_reason')}<extra></extra>"
            )
            fig.add_trace(tube)
        else:
            fig.add_trace(
                go.Scatter3d(
                    x=xs,
                    y=ys,
                    z=zs,
                    mode="lines",
                    line={"color": color, "width": 4, "dash": "dot"},
                    name=pipe_name,
                )
            )

        fig.add_trace(go.Scatter3d(x=[xs[0]], y=[ys[0]], z=[zs[0]], mode="markers", marker={"size": 5, "color": color, "symbol": "circle"}, showlegend=False))
        fig.add_trace(go.Scatter3d(x=[xs[-1]], y=[ys[-1]], z=[zs[-1]], mode="markers", marker={"size": 5, "color": color, "symbol": "diamond"}, showlegend=False))

        violations = p.get("bend_rule_violations", [])
        if violations:
            vx = [v["point"][0] for v in violations]
            vy = [v["point"][1] for v in violations]
            vz = [v["point"][2] for v in violations]
            hover_text = [
                (
                    f"pipe={p['id']}<br>"
                    f"point_index={v.get('point_index')}<br>"
                    f"estimated_radius={v.get('estimated_radius'):.3f}<br>"
                    f"required_radius={v.get('required_radius'):.3f}"
                )
                for v in violations
            ]
            fig.add_trace(
                go.Scatter3d(
                    x=vx,
                    y=vy,
                    z=vz,
                    mode="markers",
                    marker={"size": 7, "color": "red", "symbol": "x"},
                    text=hover_text,
                    hovertemplate="%{text}<extra></extra>",
                    name=f"{p['id']}_bend_violations",
                )
            )

    if case.clamp_candidates:
        cx = [c.position[0] for c in case.clamp_candidates]
        cy = [c.position[1] for c in case.clamp_candidates]
        cz = [c.position[2] for c in case.clamp_candidates]
        fig.add_trace(go.Scatter3d(x=cx, y=cy, z=cz, mode="markers", marker={"size": 4, "color": "black"}, name="clamp_candidates"))

    if routing_result.get("conflicts"):
        conflict_points_x = []
        conflict_points_y = []
        conflict_points_z = []
        by_id = {p["id"]: p for p in routing_result["pipes"]}
        for c in routing_result["conflicts"]:
            pa = by_id.get(c["pipe_a"], {})
            idx_a = c["segment_a_index"]
            if pa.get("success") and idx_a < len(pa["path"]) - 1:
                p1 = pa["path"][idx_a]
                p2 = pa["path"][idx_a + 1]
                conflict_points_x.append((p1[0] + p2[0]) / 2)
                conflict_points_y.append((p1[1] + p2[1]) / 2)
                conflict_points_z.append((p1[2] + p2[2]) / 2)
        if conflict_points_x:
            fig.add_trace(go.Scatter3d(x=conflict_points_x, y=conflict_points_y, z=conflict_points_z, mode="markers", marker={"size": 5, "color": "red"}, name="conflicts"))

    if successful_pipe_count == 0:
        fig.add_annotation(
            text="No successful pipe route found in this run.",
            x=0.5,
            y=0.5,
            xref="paper",
            yref="paper",
            showarrow=False,
            font={"size": 18, "color": "#cc0000"},
        )

    fig.update_layout(
        scene={"xaxis_title": "X", "yaxis_title": "Y", "zaxis_title": "Z"},
        title="Pipe Routing MVP",
        margin={"l": 0, "r": 0, "t": 40, "b": 0},
        height=800,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(output_path), include_plotlyjs=True, full_html=True)
