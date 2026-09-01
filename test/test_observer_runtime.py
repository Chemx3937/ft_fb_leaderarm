from ft_fb_leaderarm.contract import (
    OPERATIONAL_FREE_FORCE_HARD_MAX_LIMIT_N,
    OPERATIONAL_FREE_FORCE_P95_LIMIT_N,
    OPERATIONAL_FREE_FORCE_P99_LIMIT_N,
)
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


def observations(first, last, force_n=0.2):
    return [
        {
            "prediction_sequence": sequence,
            "contact_wrench": [force_n, 0.0, 0.0, 0.0, 0.0, 0.0],
            "contact_state": 0,
            "valid": True,
            "model_ready": True,
            "frame_id": "right_base_link",
        }
        for sequence in range(first, last + 1)
    ]


def test_observer_runtime_and_free_space_gates():
    start = snapshot(2.0, 100, 100)
    free_samples = observations(101, 2725)
    passed = analyze_observer_runtime(
        start, snapshot(12.0, 2725, 2725), free_samples
    )
    assert passed["passed"]
    assert passed["metrics"]["valid_publish_hz"] == 262.5
    assert passed["metrics"]["free_residual_force_norm_max_n"] == 0.2
    assert passed["limits"]["max_free_residual_force_p95_n"] == (
        OPERATIONAL_FREE_FORCE_P95_LIMIT_N
    )
    assert passed["limits"]["max_free_residual_force_p99_n"] == (
        OPERATIONAL_FREE_FORCE_P99_LIMIT_N
    )
    assert passed["limits"]["hard_max_free_residual_force_n"] == (
        OPERATIONAL_FREE_FORCE_HARD_MAX_LIMIT_N
    )

    quantized_duration_s = 10.999721168074757
    quantized_samples = 2887
    quantized = analyze_observer_runtime(
        start,
        snapshot(
            start["uptime_s"] + quantized_duration_s,
            start["cycles"] + quantized_samples,
            start["valid_predictions"] + quantized_samples,
        ),
        observations(101, 100 + quantized_samples),
    )
    assert quantized["gates"]["FS-05"]["passed"]
    assert quantized["metrics"]["valid_publish_hz"] < 262.5
    assert quantized["metrics"]["minimum_valid_predictions"] == quantized_samples

    runtime_rejected = analyze_observer_runtime(
        start,
        snapshot(
            12.0,
            2725,
            2724,
            invalid=1,
            deadline=1,
            reasons={"locally_stale_input": 1},
        ),
        observations(101, 2724),
    )
    assert not runtime_rejected["gates"]["FS-05"]["passed"]
    assert runtime_rejected["metrics"]["stale_publications"] == 1
    assert "valid publish rate is below 262.5 Hz" in runtime_rejected["failures"]

    contact_samples = [dict(row) for row in free_samples]
    for sample in contact_samples:
        sample["contact_wrench"] = [1.6, 0.0, 0.0, 0.0, 0.0, 0.0]
    contact_samples[-1]["contact_state"] = 1
    free_rejected = analyze_observer_runtime(
        start, snapshot(12.0, 2725, 2725), contact_samples
    )
    assert free_rejected["gates"]["FS-05"]["passed"]
    assert not free_rejected["gates"]["FS-06"]["passed"]
    assert "CONTACT samples exist in the FREE interval" in free_rejected["failures"]
    assert "FREE residual force p99 exceeds 1.5 N" in free_rejected["failures"]

    missing = analyze_observer_runtime(
        start, snapshot(12.0, 2725, 2725), free_samples[:-1]
    )
    assert "ContactObservation sequence is incomplete" in missing["failures"]
