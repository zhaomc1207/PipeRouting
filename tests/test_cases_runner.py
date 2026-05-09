from pathlib import Path
import csv
import json

from pipe_routing.io import load_routing_case
from pipe_routing.cbs import simplified_cbs
from pipe_routing.multi_pipe import route_pipes_sequentially
from pipe_routing.multi_pipe import _validate_pipe_endpoints
import run_cases


def test_all_cases_are_loadable() -> None:
    cases = sorted(Path("data/cases").glob("*.json"))
    assert len(cases) >= 7
    for p in cases:
        case = load_routing_case(p)
        assert len(case.pipes) >= 1


def test_many_pipe_cases_exist() -> None:
    cases = sorted(Path("data/cases").glob("*.json"))
    pipe_counts = []
    for p in cases:
        case = load_routing_case(p)
        pipe_counts.append(len(case.pipes))
    assert any(n >= 6 for n in pipe_counts)
    assert any(n >= 10 for n in pipe_counts)


def test_run_cases_generates_summary_csv() -> None:
    run_cases.main()
    summary = Path("outputs/cases/summary.csv")
    assert summary.exists()
    assert summary.stat().st_size > 0
    rows = list(csv.DictReader(summary.read_text(encoding="utf-8").splitlines()))
    assert rows
    first = rows[0]
    assert "result_json_path" in first
    assert "result_html_path" in first
    assert "routing_report_path" in first
    assert "pipe_count" in first
    assert "success_rate" in first
    assert "degraded_pipe_count" in first
    assert "invalid_pipe_ids" in first
    assert "invalid_reason" in first
    assert "all_quality_checks_passed" in first
    assert "runtime_seconds" in first
    for row in rows:
        assert Path(row["result_json_path"]).exists()
        assert Path(row["result_html_path"]).exists()
        assert Path(row["routing_report_path"]).exists()
        case_name = row["case_name"]
        case_dir = Path("outputs/cases") / case_name
        assert (case_dir / f"{case_name}_result.json").exists()
        assert (case_dir / f"{case_name}_result.html").exists()
        assert (case_dir / f"{case_name}_routing_report.md").exists()
        assert (case_dir / "result.json").exists()
        assert (case_dir / "result.html").exists()
        assert (case_dir / "routing_report.md").exists()
        assert "input_invalid" in row
        assert "degraded_pipe_count" in row


def test_no_solution_case_does_not_crash_and_has_failure() -> None:
    case = load_routing_case(Path("data/cases/case_07_no_solution.json"))
    initial = route_pipes_sequentially(case)
    result = simplified_cbs(case, initial, max_iterations=3, padding=40.0)
    assert any(not p["success"] for p in result["pipes"])


def test_many_pipe_case_partial_failure_does_not_crash() -> None:
    row = run_cases.run_case(Path("data/cases/case_10_many_pipes_dense_10.json"), Path("outputs/cases"))
    assert row["case_name"] == "case_10_many_pipes_dense_10"
    assert int(row["pipe_count"]) >= 10
    assert int(row["success_count"]) + int(row["failed_count"]) == int(row["pipe_count"])


def test_dense_obstacles_case_has_no_bend_violations() -> None:
    row = run_cases.run_case(Path("data/cases/case_06_dense_obstacles.json"), Path("outputs/cases"))
    data = json.loads(Path(row["result_json_path"]).read_text(encoding="utf-8"))
    assert all(
        p.get("bend_rule_violation_count", 0) == 0
        for p in data.get("pipes", [])
        if p.get("success")
    )


def test_case03_endpoints_are_not_blocked_by_inflated_obstacles() -> None:
    case = load_routing_case(Path("data/cases/case_03_multi_pipe_conflict.json"))
    issues = _validate_pipe_endpoints(case)
    assert issues == {}


def test_case03_has_no_start_blocked_error() -> None:
    row = run_cases.run_case(Path("data/cases/case_03_multi_pipe_conflict.json"), Path("outputs/cases"))
    data = json.loads(Path(row["result_json_path"]).read_text(encoding="utf-8"))
    assert all("Start point blocked by obstacle." not in str(p.get("error")) for p in data.get("pipes", []))


def test_case03_summary_input_invalid_false() -> None:
    run_cases.main()
    rows = list(csv.DictReader(Path("outputs/cases/summary.csv").read_text(encoding="utf-8").splitlines()))
    row = next(r for r in rows if r["case_name"] == "case_03_multi_pipe_conflict")
    assert row["input_invalid"].lower() == "false"


def test_case09_endpoints_are_valid() -> None:
    case = load_routing_case(Path("data/cases/case_09_many_pipes_10.json"))
    issues = _validate_pipe_endpoints(case)
    assert issues == {}


def test_non_no_solution_cases_no_obstacle_start_blocked_error() -> None:
    for case_path in sorted(Path("data/cases").glob("*.json")):
        if "no_solution" in case_path.stem:
            continue
        row = run_cases.run_case(case_path, Path("outputs/cases"))
        data = json.loads(Path(row["result_json_path"]).read_text(encoding="utf-8"))
        for p in data.get("pipes", []):
            assert "Start point blocked by obstacle." not in str(p.get("error"))


def test_case08_conflicts_stay_zero() -> None:
    row = run_cases.run_case(Path("data/cases/case_08_many_pipes_6.json"), Path("outputs/cases"))
    assert int(row["total_conflicts"]) == 0
    assert int(row["total_bend_violations"]) == 0


def test_case09_conflicts_stay_zero() -> None:
    row = run_cases.run_case(Path("data/cases/case_09_many_pipes_10.json"), Path("outputs/cases"))
    assert int(row["total_conflicts"]) == 0
    assert int(row["total_bend_violations"]) == 0


def test_case10_conflicts_below_previous_baseline() -> None:
    row = run_cases.run_case(Path("data/cases/case_10_many_pipes_dense_10.json"), Path("outputs/cases"))
    assert int(row["total_conflicts"]) < 6
