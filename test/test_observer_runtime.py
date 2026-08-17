from ft_fb_leaderarm.observer_runtime import analyze_observer_runtime


def snapshot(
    uptime_s,
    cycles,
    valid,
    invalid=0,
    deadline=0,
    failures=0,
    reasons=None,
):
    return {
        "approved_model": True,
        "model_ready": True,
        "baseline_ready": True,
        "observer_ready": True,
        "sample_hz": 262.5,
        "uptime_s": uptime_s,
        "model_sha256": "a" * 64,
        "zero_set_id": "zero-1",
        "payload_id": "payload-1",
        "controller_config_hash": "controller-1",
        "cycles": cycles,
        "valid_predictions": valid,
        "invalid_publications": invalid,
        "deadline_misses": deadline,
        "inference_failures": failures,
        "invalid_reason_counts": reasons or {},
    }


def test_observer_runtime_gate_passes_exact_rate_and_rejects_failures():
    start = snapshot(2.0, 100, 100)
    passed = analyze_observer_runtime(start, snapshot(12.0, 2725, 2725))
    assert passed["passed"]
    assert passed["metrics"]["valid_publish_hz"] == 262.5

    rejected = analyze_observer_runtime(
        start,
        snapshot(
            12.0,
            2725,
            2724,
            invalid=1,
            deadline=1,
            reasons={"locally_stale_input": 1},
        ),
    )
    assert not rejected["passed"]
    assert rejected["metrics"]["stale_publications"] == 1
    assert "valid publish rate is below 262.5 Hz" in rejected["failures"]
