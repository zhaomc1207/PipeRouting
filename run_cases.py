from __future__ import annotations

import csv
import json
from pathlib import Path

from pipe_routing.cbs import simplified_cbs
from pipe_routing.io import load_routing_case
from pipe_routing.multi_pipe import route_pipes_sequentially
from pipe_routing.visualize import write_result_html


def _write_routing_report(result: dict, report_path: Path) -> None:
    pipes = result["pipes"]
    success_count = sum(1 for p in pipes if p["success"])
    total_length = sum(float(p.get("length", 0.0)) for p in pipes if p["success"])
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
    for p in pipes:
        lines.append(f"### {p['id']}")
        lines.append(f"- success: {p['success']}")
        lines.append(f"- length: {p.get('length', 0.0):.3f}")
        lines.append(f"- conflict_count: {p.get('conflict_count', 0)}")
        lines.append(f"- bend_rule_violation_count: {p.get('bend_rule_violation_count', 0)}")
        lines.append("")
    report_path.write_text("\n".join(lines), encoding="utf-8")


def run_case(case_path: Path, output_root: Path) -> dict:
    case = load_routing_case(case_path)
    initial = route_pipes_sequentially(case)
    result = simplified_cbs(case, initial, max_iterations=5, padding=40.0)

    case_dir = output_root / case_path.stem
    case_dir.mkdir(parents=True, exist_ok=True)
    case_name = case_path.stem
    prefixed_result_json = case_dir / f"{case_name}_result.json"
    prefixed_result_html = case_dir / f"{case_name}_result.html"
    prefixed_report_md = case_dir / f"{case_name}_routing_report.md"
    legacy_result_json = case_dir / "result.json"
    legacy_result_html = case_dir / "result.html"
    legacy_report_md = case_dir / "routing_report.md"

    payload = json.dumps(result, ensure_ascii=False, indent=2)
    prefixed_result_json.write_text(payload, encoding="utf-8")
    legacy_result_json.write_text(payload, encoding="utf-8")
    write_result_html(case, result, prefixed_result_html)
    write_result_html(case, result, legacy_result_html)
    _write_routing_report(result, prefixed_report_md)
    _write_routing_report(result, legacy_report_md)

    pipes = result["pipes"]
    total_pipes = len(pipes)
    success_count = sum(1 for p in pipes if p["success"])
    failed_count = total_pipes - success_count
    total_conflicts = len(result.get("conflicts", []))
    total_length = sum(float(p.get("length", 0.0)) for p in pipes if p["success"])
    total_bend_violations = sum(int(p.get("bend_rule_violation_count", 0)) for p in pipes if p["success"])
    notes = "has_failure" if failed_count > 0 else "ok"

    return {
        "case_name": case_name,
        "total_pipes": total_pipes,
        "success_count": success_count,
        "failed_count": failed_count,
        "total_conflicts": total_conflicts,
        "total_length": f"{total_length:.3f}",
        "total_bend_violations": total_bend_violations,
        "notes": notes,
        "result_json_path": str(prefixed_result_json.relative_to(output_root.parent.parent)),
        "result_html_path": str(prefixed_result_html.relative_to(output_root.parent.parent)),
        "routing_report_path": str(prefixed_report_md.relative_to(output_root.parent.parent)),
    }


def main() -> None:
    root = Path(__file__).resolve().parent
    cases_dir = root / "data" / "cases"
    output_root = root / "outputs" / "cases"
    output_root.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    for case_path in sorted(cases_dir.glob("*.json")):
        rows.append(run_case(case_path, output_root))

    summary_path = output_root / "summary.csv"
    fields = [
        "case_name",
        "total_pipes",
        "success_count",
        "failed_count",
        "total_conflicts",
        "total_length",
        "total_bend_violations",
        "notes",
        "result_json_path",
        "result_html_path",
        "routing_report_path",
    ]
    with summary_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    print(f"cases_processed={len(rows)} summary={summary_path}")


if __name__ == "__main__":
    main()
