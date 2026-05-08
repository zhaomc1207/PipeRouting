from __future__ import annotations

import json
from pathlib import Path

from pipe_routing.cbs import simplified_cbs
from pipe_routing.io import load_routing_case
from pipe_routing.local_reroute import local_reroute
from pipe_routing.multi_pipe import route_pipes_sequentially
from pipe_routing.visualize import write_result_html


def _build_clamp_usage(case, result: dict) -> dict:
    by_pipe = {p["id"]: p for p in result["pipes"]}
    usage: dict = {}
    for clamp in case.clamp_candidates:
        applies_to = clamp.applies_to[:] if clamp.applies_to else [p.id for p in case.pipes]
        used_by: list[str] = []
        for pid in applies_to:
            pipe = by_pipe.get(pid)
            if pipe and clamp.id in pipe.get("used_clamps", []):
                used_by.append(pid)
        usage[clamp.id] = {
            "applies_to": applies_to,
            "used_by": used_by,
            "unused_for": [pid for pid in applies_to if pid not in used_by],
            "radius": clamp.radius,
            "position": [clamp.position[0], clamp.position[1], clamp.position[2]],
        }
    return usage


def _write_routing_report(result: dict, report_path: Path) -> None:
    pipes = result["pipes"]
    success_count = sum(1 for p in pipes if p["success"])
    total_length = sum(float(p["length"]) for p in pipes if p["success"])
    total_conflicts = len(result.get("conflicts", []))

    lines: list[str] = []
    lines.append("# Routing Report")
    lines.append("")
    lines.append("## Summary")
    lines.append(f"- total_pipes: {len(pipes)}")
    lines.append(f"- success_count: {success_count}")
    lines.append(f"- failed_count: {len(pipes) - success_count}")
    lines.append(f"- total_length: {total_length:.3f}")
    lines.append(f"- total_conflicts: {total_conflicts}")
    lines.append("- optimization_priority_note: length is a weak objective in current phase")
    lines.append(f"- cbs_enabled: {result.get('cbs_enabled')}")
    lines.append(f"- cbs_iterations: {result.get('cbs_iterations')}")
    lines.append(f"- cbs_resolved_conflicts: {result.get('cbs_resolved_conflicts')}")
    lines.append(f"- cbs_remaining_conflicts: {result.get('cbs_remaining_conflicts')}")
    lines.append("")
    lines.append("## CBS")
    lines.append("")
    for log in result.get("cbs_log", []):
        lines.append(f"- {log}")
    lines.append("")
    lines.append("## Pipes")
    lines.append("")
    lines.append("- Note: current phase prioritizes feasibility/conflict-free/smoothness over shortest length.")
    lines.append("- length is reported as reference metric, not primary optimization target.")
    lines.append("")

    for pipe in pipes:
        lines.append(f"### {pipe['id']}")
        lines.append(f"- success: {pipe['success']}")
        lines.append(f"- length: {pipe['length']:.3f}")
        lines.append(f"- bend_count: {pipe['bend_count']}")
        lines.append(f"- min_bend_radius_ok: {pipe.get('min_bend_radius_ok', True)}")
        lines.append(f"- bend_radius_violation_count: {pipe.get('bend_radius_violation_count', 0)}")
        lines.append(f"- min_bend_radius_required: {pipe.get('min_bend_radius_required')}")
        lines.append(f"- min_bend_radius_observed: {pipe.get('min_bend_radius_observed')}")
        lines.append(f"- bend_rule_violation_count: {pipe.get('bend_rule_violation_count', 0)}")
        lines.append(f"- conflict_count: {pipe['conflict_count']}")
        lines.append(f"- smoothing_applied: {pipe.get('smoothing_applied', False)}")
        lines.append(f"- smoothing_reverted: {pipe.get('smoothing_reverted', False)}")
        lines.append(f"- smoothing_revert_reason: {pipe.get('smoothing_revert_reason')}")
        lines.append(f"- raw_point_count: {len(pipe.get('raw_path', []))}")
        lines.append(f"- smoothed_point_count: {len(pipe.get('smoothed_path', []))}")
        lines.append(f"- rerouted: {pipe.get('rerouted', False)}")
        lines.append(f"- affected_by_changed_region: {pipe.get('affected_by_changed_region', False)}")
        lines.append(f"- cbs_rerouted: {pipe.get('cbs_rerouted', False)}")
        lines.append(f"- cbs_constraints_count: {pipe.get('cbs_constraints_count', 0)}")
        lines.append(f"- cbs_reroute_reason: {pipe.get('cbs_reroute_reason')}")
        own = []
        for cid, cu in result.get("clamp_usage", {}).items():
            if pipe["id"] in cu.get("applies_to", []):
                own.append(cid)
        used = list(pipe.get("used_clamps", []))
        missed = [cid for cid in own if cid not in used]
        lines.append(f"- own_clamps: {own}")
        lines.append(f"- used_clamps: {pipe.get('used_clamps', [])}")
        lines.append(f"- missed_clamps: {missed}")
        lines.append(f"- min_distance_to_clamps: {pipe.get('min_distance_to_clamps', {})}")
        if not pipe["success"]:
            lines.append(f"- error: {pipe.get('error')}")
        lines.append("")

    lines.append("## Clamp Relationships")
    lines.append("")
    for cid, cu in result.get("clamp_usage", {}).items():
        lines.append(f"### {cid}")
        lines.append(f"- applies_to: {cu.get('applies_to', [])}")
        lines.append(f"- used_by: {cu.get('used_by', [])}")
        lines.append(f"- unused_for: {cu.get('unused_for', [])}")
        lines.append("")

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    root = Path(__file__).resolve().parent
    input_path = root / "data" / "demo_case.json"
    output_dir = root / "outputs"
    output_dir.mkdir(parents=True, exist_ok=True)

    case = load_routing_case(input_path)
    initial_result = route_pipes_sequentially(case)
    result = simplified_cbs(case, initial_result, max_iterations=5, padding=40.0)
    result["clamp_usage"] = _build_clamp_usage(case, result)

    json_path = output_dir / "result.json"
    html_path = output_dir / "result.html"
    report_path = output_dir / "routing_report.md"

    with json_path.open("w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    write_result_html(case, result, html_path)
    _write_routing_report(result, report_path)

    for pipe in result["pipes"]:
        if pipe["success"]:
            print(
                f"{pipe['id']} success=True length={pipe['length']:.1f} "
                f"bend_count={pipe['bend_count']} conflict_count={pipe['conflict_count']}"
            )
        else:
            print(f"{pipe['id']} success=False error=\"{pipe['error']}\"")
    print(f"total_conflicts={len(result.get('conflicts', []))}")


if __name__ == "__main__":
    main()


def demo_local_reroute(changed_region: dict) -> dict:
    """Optional helper for phase2 local reroute demo without changing default behavior."""
    root = Path(__file__).resolve().parent
    case = load_routing_case(root / "data" / "demo_case.json")
    base = route_pipes_sequentially(case)
    return local_reroute(case, base, changed_region)
