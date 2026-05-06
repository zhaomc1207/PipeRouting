import json
from pathlib import Path

import run_demo


def test_run_demo_generates_routing_report() -> None:
    run_demo.main()
    report_path = Path("outputs/routing_report.md")
    assert report_path.exists()
    text = report_path.read_text(encoding="utf-8")
    assert "total_pipes" in text
    assert "success_count" in text
    assert "used_clamps" in text
    assert "min_distance_to_clamps" in text
    assert "smoothing_applied" in text
    assert "smoothing_reverted" in text
    assert "raw_point_count" in text
    assert "smoothed_point_count" in text
    assert "min_bend_radius_required" in text
    assert "min_bend_radius_observed" in text
    assert "bend_rule_violation_count" in text

    result = json.loads(Path("outputs/result.json").read_text(encoding="utf-8"))
    for pipe in result["pipes"]:
        assert "min_bend_radius_required" in pipe
        assert "min_bend_radius_observed" in pipe
        assert "bend_rule_violation_count" in pipe
        assert "bend_rule_violations" in pipe
