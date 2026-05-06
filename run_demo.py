from __future__ import annotations

import json
from pathlib import Path

from pipe_routing.io import load_routing_case
from pipe_routing.multi_pipe import route_pipes_sequentially
from pipe_routing.visualize import write_result_html


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
    lines.append("")
    lines.append("## Pipes")
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
        lines.append(f"- used_clamps: {pipe.get('used_clamps', [])}")
        lines.append(f"- min_distance_to_clamps: {pipe.get('min_distance_to_clamps', {})}")
        if not pipe["success"]:
            lines.append(f"- error: {pipe.get('error')}")
        lines.append("")

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    root = Path(__file__).resolve().parent
    input_path = root / "data" / "demo_case.json"
    output_dir = root / "outputs"
    output_dir.mkdir(parents=True, exist_ok=True)

    case = load_routing_case(input_path)
    result = route_pipes_sequentially(case)

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
