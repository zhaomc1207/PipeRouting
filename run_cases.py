from __future__ import annotations

import csv
import json
import time
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
    lines.append(f"- unresolved_conflicts: {total_conflicts > 0}")
    lines.append(f"- cbs_iterations: {result.get('cbs_iterations', 0)}")
    lines.append(f"- cbs_attempts: {result.get('cbs_attempts', 0)}")
    lines.append(f"- cbs_resolved_conflicts: {result.get('cbs_resolved_conflicts', 0)}")
    lines.append(f"- cbs_remaining_conflicts: {result.get('cbs_remaining_conflicts', 0)}")
    lines.append(f"- input_invalid: {result.get('input_invalid', False)}")
    lines.append(f"- invalid_pipe_ids: {result.get('invalid_pipe_ids', [])}")
    lines.append(f"- invalid_reason: {result.get('invalid_reason', '')}")
    lines.append("")
    lines.append("## CBS")
    lines.append("")
    for log in result.get("cbs_log", []):
        lines.append(f"- {log}")
    lines.append("")
    for p in pipes:
        lines.append(f"### {p['id']}")
        lines.append(f"- success: {p['success']}")
        lines.append(f"- length: {p.get('length', 0.0):.3f}")
        lines.append(f"- conflict_count: {p.get('conflict_count', 0)}")
        lines.append(f"- bend_rule_violation_count: {p.get('bend_rule_violation_count', 0)}")
        lines.append(f"- input_invalid: {p.get('input_invalid', False)}")
        lines.append(f"- degraded_result: {p.get('degraded_result', False)}")
        lines.append(f"- error: {p.get('error')}")
        lines.append("")
    report_path.write_text("\n".join(lines), encoding="utf-8")


def run_case(case_path: Path, output_root: Path) -> dict:
    t0 = time.perf_counter()
    case = load_routing_case(case_path)
    initial = route_pipes_sequentially(case)
    result = simplified_cbs(case, initial, max_iterations=20, padding=40.0)
    result["input_invalid"] = bool(initial.get("input_invalid", False))
    result["invalid_pipe_ids"] = list(initial.get("invalid_pipe_ids", []))
    result["invalid_reason"] = str(initial.get("invalid_reason", ""))

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
    input_invalid = bool(result.get("input_invalid", False))
    invalid_pipe_ids = list(result.get("invalid_pipe_ids", []))
    invalid_reason = str(result.get("invalid_reason", ""))
    degraded_pipe_count = sum(1 for p in pipes if bool(p.get("degraded_result", False)))
    success_rate = (success_count / total_pipes) if total_pipes > 0 else 0.0
    all_quality_checks_passed = (
        failed_count == 0
        and total_conflicts == 0
        and total_bend_violations == 0
        and degraded_pipe_count == 0
        and not input_invalid
    )
    runtime_seconds = time.perf_counter() - t0
    notes = "has_failure" if failed_count > 0 else "ok"

    return {
        "case_name": case_name,
        "pipe_count": total_pipes,
        "total_pipes": total_pipes,
        "success_count": success_count,
        "failed_count": failed_count,
        "success_rate": f"{success_rate:.4f}",
        "total_conflicts": total_conflicts,
        "total_length": f"{total_length:.3f}",
        "total_bend_violations": total_bend_violations,
        "degraded_pipe_count": degraded_pipe_count,
        "input_invalid": input_invalid,
        "invalid_pipe_ids": "|".join(invalid_pipe_ids),
        "invalid_reason": invalid_reason,
        "all_quality_checks_passed": all_quality_checks_passed,
        "unresolved_conflicts": total_conflicts > 0,
        "cbs_iterations": int(result.get("cbs_iterations", 0)),
        "cbs_remaining_conflicts": int(result.get("cbs_remaining_conflicts", total_conflicts)),
        "runtime_seconds": f"{runtime_seconds:.3f}",
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
        "pipe_count",
        "total_pipes",
        "success_count",
        "failed_count",
        "success_rate",
        "total_conflicts",
        "total_length",
        "total_bend_violations",
        "degraded_pipe_count",
        "input_invalid",
        "invalid_pipe_ids",
        "invalid_reason",
        "all_quality_checks_passed",
        "unresolved_conflicts",
        "cbs_iterations",
        "cbs_remaining_conflicts",
        "runtime_seconds",
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
