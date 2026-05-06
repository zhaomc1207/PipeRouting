from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class Workspace:
    min_corner: Vec3
    max_corner: Vec3
    resolution: float


@dataclass(frozen=True)
class Obstacle:
    id: str
    type: str
    min_corner: Vec3
    max_corner: Vec3


@dataclass(frozen=True)
class Pipe:
    id: str
    start: Vec3
    end: Vec3
    diameter: float
    clearance: float
    min_bend_radius: float


@dataclass(frozen=True)
class ClampCandidate:
    id: str
    position: Vec3
    radius: float


@dataclass(frozen=True)
class RoutingCase:
    workspace: Workspace
    obstacles: list[Obstacle]
    pipes: list[Pipe]
    clamp_candidates: list[ClampCandidate]


def _require_keys(payload: dict[str, Any], keys: list[str], section: str) -> None:
    for key in keys:
        if key not in payload:
            raise ValueError(f"Missing key '{key}' in {section}.")


def _to_vec3(value: Any, field_name: str) -> Vec3:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"Field '{field_name}' must be a 3-element list.")
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Field '{field_name}' contains non-numeric values.") from exc


def _parse_workspace(payload: dict[str, Any]) -> Workspace:
    _require_keys(payload, ["min", "max", "resolution"], "workspace")
    ws = Workspace(
        min_corner=_to_vec3(payload["min"], "workspace.min"),
        max_corner=_to_vec3(payload["max"], "workspace.max"),
        resolution=float(payload["resolution"]),
    )
    if ws.resolution <= 0:
        raise ValueError("workspace.resolution must be > 0.")
    if any(a >= b for a, b in zip(ws.min_corner, ws.max_corner)):
        raise ValueError("workspace.min must be strictly smaller than workspace.max on every axis.")
    return ws


def _parse_obstacles(payload: Any) -> list[Obstacle]:
    if not isinstance(payload, list):
        raise ValueError("obstacles must be a list.")
    obstacles: list[Obstacle] = []
    for idx, item in enumerate(payload):
        if not isinstance(item, dict):
            raise ValueError(f"obstacles[{idx}] must be an object.")
        _require_keys(item, ["id", "type", "min", "max"], f"obstacles[{idx}]")
        if item["type"] != "box":
            raise ValueError(f"obstacles[{idx}].type only supports 'box' in MVP.")
        min_corner = _to_vec3(item["min"], f"obstacles[{idx}].min")
        max_corner = _to_vec3(item["max"], f"obstacles[{idx}].max")
        if any(a >= b for a, b in zip(min_corner, max_corner)):
            raise ValueError(f"obstacles[{idx}] min must be smaller than max on every axis.")
        obstacles.append(
            Obstacle(
                id=str(item["id"]),
                type="box",
                min_corner=min_corner,
                max_corner=max_corner,
            )
        )
    return obstacles


def _parse_pipes(payload: Any) -> list[Pipe]:
    if not isinstance(payload, list):
        raise ValueError("pipes must be a list.")
    pipes: list[Pipe] = []
    for idx, item in enumerate(payload):
        if not isinstance(item, dict):
            raise ValueError(f"pipes[{idx}] must be an object.")
        _require_keys(
            item,
            ["id", "start", "end", "diameter", "clearance", "min_bend_radius"],
            f"pipes[{idx}]",
        )
        pipe = Pipe(
            id=str(item["id"]),
            start=_to_vec3(item["start"], f"pipes[{idx}].start"),
            end=_to_vec3(item["end"], f"pipes[{idx}].end"),
            diameter=float(item["diameter"]),
            clearance=float(item["clearance"]),
            min_bend_radius=float(item["min_bend_radius"]),
        )
        if pipe.diameter <= 0:
            raise ValueError(f"pipes[{idx}].diameter must be > 0.")
        if pipe.clearance < 0:
            raise ValueError(f"pipes[{idx}].clearance must be >= 0.")
        if pipe.min_bend_radius < 0:
            raise ValueError(f"pipes[{idx}].min_bend_radius must be >= 0.")
        pipes.append(pipe)
    return pipes


def _parse_clamps(payload: Any) -> list[ClampCandidate]:
    if not isinstance(payload, list):
        raise ValueError("clamp_candidates must be a list.")
    clamps: list[ClampCandidate] = []
    for idx, item in enumerate(payload):
        if not isinstance(item, dict):
            raise ValueError(f"clamp_candidates[{idx}] must be an object.")
        _require_keys(item, ["id", "position", "radius"], f"clamp_candidates[{idx}]")
        radius = float(item["radius"])
        if radius < 0:
            raise ValueError(f"clamp_candidates[{idx}].radius must be >= 0.")
        clamps.append(
            ClampCandidate(
                id=str(item["id"]),
                position=_to_vec3(item["position"], f"clamp_candidates[{idx}].position"),
                radius=radius,
            )
        )
    return clamps


def load_routing_case(json_path: Path) -> RoutingCase:
    """Load and validate routing input json."""
    if not json_path.exists():
        raise FileNotFoundError(f"Input json not found: {json_path}")

    with json_path.open("r", encoding="utf-8-sig") as f:
        payload = json.load(f)

    if not isinstance(payload, dict):
        raise ValueError("Root JSON value must be an object.")

    _require_keys(payload, ["workspace", "obstacles", "pipes", "clamp_candidates"], "root")

    return RoutingCase(
        workspace=_parse_workspace(payload["workspace"]),
        obstacles=_parse_obstacles(payload["obstacles"]),
        pipes=_parse_pipes(payload["pipes"]),
        clamp_candidates=_parse_clamps(payload["clamp_candidates"]),
    )
