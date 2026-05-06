from __future__ import annotations

from pathlib import Path

import plotly.graph_objects as go

from .io import RoutingCase


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


def write_result_html(case: RoutingCase, routing_result: dict, output_path: Path) -> None:
    fig = go.Figure()
    successful_pipe_count = 0

    for obs in case.obstacles:
        fig.add_trace(_box_mesh(obs.min_corner, obs.max_corner))

    colors = ["#1f77b4", "#2ca02c", "#ff7f0e", "#d62728", "#17becf", "#8c564b"]
    for idx, p in enumerate(routing_result["pipes"]):
        if not p["success"] or not p["path"]:
            continue
        successful_pipe_count += 1
        pts = p["path"]
        xs = [a[0] for a in pts]
        ys = [a[1] for a in pts]
        zs = [a[2] for a in pts]
        color = colors[idx % len(colors)]
        fig.add_trace(go.Scatter3d(x=xs, y=ys, z=zs, mode="lines", line={"color": color, "width": 6}, name=p["id"]))
        fig.add_trace(go.Scatter3d(x=[xs[0]], y=[ys[0]], z=[zs[0]], mode="markers", marker={"size": 5, "color": color, "symbol": "circle"}, showlegend=False))
        fig.add_trace(go.Scatter3d(x=[xs[-1]], y=[ys[-1]], z=[zs[-1]], mode="markers", marker={"size": 5, "color": color, "symbol": "diamond"}, showlegend=False))

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
