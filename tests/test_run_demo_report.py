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
    assert "rerouted" in text
    assert "affected_by_changed_region" in text
    assert "min_bend_radius_required" in text
    assert "min_bend_radius_observed" in text
    assert "bend_rule_violation_count" in text
    assert "cbs_enabled" in text
    assert "cbs_iterations" in text
    assert "cbs_resolved_conflicts" in text
    assert "cbs_remaining_conflicts" in text
    assert "own_clamps" in text
    assert "missed_clamps" in text

    result = json.loads(Path("outputs/result.json").read_text(encoding="utf-8"))
    assert "cbs_enabled" in result
    assert "cbs_iterations" in result
    assert "cbs_resolved_conflicts" in result
    assert "cbs_remaining_conflicts" in result
    assert "cbs_log" in result
    assert "clamp_usage" in result
    for pipe in result["pipes"]:
        assert "min_bend_radius_required" in pipe
        assert "min_bend_radius_observed" in pipe
        assert "bend_rule_violation_count" in pipe
        assert "bend_rule_violations" in pipe
        assert "cbs_rerouted" in pipe
        assert "cbs_constraints_count" in pipe
        assert "cbs_reroute_reason" in pipe
        assert "used_clamps" in pipe
