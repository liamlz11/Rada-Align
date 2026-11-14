import json

import pytest

from rada_align.config import ERROR_SOURCES, load_config, with_only_sources
from rada_align.estimator import ERROR_AXES
from rada_align.montecarlo import (
    PRELIMINARY_BUDGET_DEG,
    format_budget,
    format_summary,
    run_montecarlo,
    sensitivity,
    summarize,
)
from rada_align.sim import main

CFG = load_config()
QUIET = with_only_sources(CFG, ())


def test_montecarlo_is_reproducible_and_summarized():
    first = run_montecarlo(QUIET, runs=4, seed=3)
    again = run_montecarlo(QUIET, runs=4, seed=3)
    assert [r.truth_verify for r in first] == [r.truth_verify for r in again]

    summary = summarize(first, CFG.geometry.tolerance_deg)
    assert summary["runs"] == 4
    assert summary["reported_pass_rate"] == 1.0
    assert summary["true_pass_rate"] == 1.0
    assert summary["false_accept_rate"] == 0.0
    assert not summary["stop_gate_p95_exceeds_tolerance"]
    for axis in ERROR_AXES:
        assert summary["axes"][axis]["true_residual_p95_deg"] < CFG.geometry.tolerance_deg
    assert "STOP gate" in format_summary(summary)


def test_stop_gate_triggers_on_a_large_unseen_bias():
    # A 0.2 deg calibration error is invisible to the system but not to the truth.
    biased = QUIET.replace_path("calibration.residual_sigma_deg", 0.2)
    summary = summarize(run_montecarlo(biased, runs=6, seed=3), CFG.geometry.tolerance_deg)
    assert summary["reported_pass_rate"] == 1.0
    assert summary["true_pass_rate"] < 1.0
    assert summary["false_accept_rate"] > 0.0
    assert summary["stop_gate_p95_exceeds_tolerance"]


def test_budget_table_covers_every_source():
    assert set(PRELIMINARY_BUDGET_DEG) == set(ERROR_SOURCES)
    rows = sensitivity(CFG, runs=2, seed=3)
    assert list(rows) == ["none", *ERROR_SOURCES, "all"]
    assert rows["calibration"]["preliminary_deg"] == 0.015
    assert "calibration" in format_budget(rows)


def test_cli_writes_report_and_plots(tmp_path, capsys):
    code = main(["--runs", "3", "--sens-runs", "2", "--out", str(tmp_path), "--seed", "1"])
    assert code == 0
    text = capsys.readouterr().out
    assert "SIMULATION with ASSUMED parameters" in text
    report = json.loads((tmp_path / "montecarlo.json").read_text(encoding="utf-8"))
    assert report["summary"]["runs"] == 3
    for name in ("convergence.png", "residual_histogram.png", "error_budget.png"):
        assert (tmp_path / name).stat().st_size > 0


def test_cli_stream_emits_json_lines(capsys):
    assert main(["--stream", "--seed", "1", "--track-time", "1"]) == 0
    lines = capsys.readouterr().out.strip().splitlines()
    messages = [json.loads(line) for line in lines]
    assert messages[0]["sim"] is True
    assert set(messages[0]["truth"]) == set(ERROR_AXES)
    states = {m["state"] for m in messages}
    assert "ACQUIRE" in states and "ALIGN" in states
    assert messages[-1]["state"] in ("TRACK", "FAULT", "VERIFY", "ALIGN")
    assert pytest.approx(messages[1]["t_s"] - messages[0]["t_s"]) == 1.0 / CFG.camera.rate_hz
