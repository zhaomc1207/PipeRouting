from pathlib import Path
import csv
import json

from pipe_routing.io import load_routing_case
from pipe_routing.cbs import simplified_cbs
from pipe_routing.multi_pipe import route_pipes_sequentially
import run_cases


def test_all_cases_are_loadable() -> None:
    cases = sorted(Path("data/cases").glob("*.json"))
    assert len(cases) >= 7
    for p in cases:
        case = load_routing_case(p)
        assert len(case.pipes) >= 1


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


def test_no_solution_case_does_not_crash_and_has_failure() -> None:
    case = load_routing_case(Path("data/cases/case_07_no_solution.json"))
    initial = route_pipes_sequentially(case)
    result = simplified_cbs(case, initial, max_iterations=3, padding=40.0)
    assert any(not p["success"] for p in result["pipes"])


def test_dense_obstacles_case_has_no_bend_violations() -> None:
    row = run_cases.run_case(Path("data/cases/case_06_dense_obstacles.json"), Path("outputs/cases"))
    data = json.loads(Path(row["result_json_path"]).read_text(encoding="utf-8"))
    assert all(
        p.get("bend_rule_violation_count", 0) == 0
        for p in data.get("pipes", [])
        if p.get("success")
    )
