import numpy as np

from ft_fb_leaderarm.collector_node import (
    _fresh_fast_state,
    _teleop_state_from_json,
)
from ft_fb_leaderarm.contract import (
    BASE_FEATURE_DIM,
    CausalFeatureBuilder,
    DEFAULT_ZERO_POSE_DEG,
    FixedPoseZeroVerifier,
    RateGate,
    SAMPLE_HZ,
    SchmittContactDetector,
    error_metrics,
    episode_timing_quality,
    project_feature_windows,
    sensor_wrench_to_base,
)


def test_rate_gate_selects_262p5_hz_from_1khz():
    gate = RateGate(SAMPLE_HZ)
    stamps = 10.0 + np.arange(2001) * 0.001
    selected = np.asarray([stamp for stamp in stamps if gate.accept(stamp)])
    assert 524 <= len(selected) <= 526
    assert abs((len(selected) - 1) / (selected[-1] - selected[0]) - SAMPLE_HZ) < 1.0


def test_causal_feature_builder_uses_only_previous_velocity():
    builder = CausalFeatureBuilder()
    first = builder.build(np.zeros(6), np.zeros(6), 1.0)
    second = builder.build(np.ones(6), np.full(6, 0.1), 1.01)
    assert first.shape == (BASE_FEATURE_DIM,)
    assert np.allclose(first[12:], 0.0)
    assert np.allclose(second[12:], 10.0)


def test_fixed_pose_zero_verifier_accepts_stable_zero_and_rejects_motion():
    verifier = FixedPoseZeroVerifier(settle_s=0.1, minimum_samples=8)
    q = np.deg2rad(DEFAULT_ZERO_POSE_DEG)
    for index in range(12):
        ready = verifier.update(
            1.0 + 0.01 * index,
            q,
            np.zeros(6),
            np.asarray([0.1, -0.1, 0.05, 0.0, 0.0, 0.0]),
        )
    assert ready
    assert verifier.status() == (True, "zero_verified")
    verifier.update(2.0, q, np.ones(6), np.zeros(6))
    assert verifier.status()[0] is False


def test_default_zero_verifier_accepts_aft_force_std_below_0p4_n():
    verifier = FixedPoseZeroVerifier(settle_s=0.1, minimum_samples=8)
    q = np.deg2rad(DEFAULT_ZERO_POSE_DEG)
    for index in range(12):
        ready = verifier.update(
            1.0 + 0.01 * index,
            q,
            np.zeros(6),
            np.asarray([0.3 if index % 2 else -0.3, 0.0, 0.0, 0.0, 0.0, 0.0]),
        )
    assert ready


def test_ablation_projection_shapes():
    windows = np.zeros((5, 16, BASE_FEATURE_DIM), dtype=np.float32)
    assert project_feature_windows(windows[:, -1:], "static").shape == (5, 12)
    assert project_feature_windows(windows[:, -1:], "dynamic").shape == (5, 24)
    assert project_feature_windows(windows, "history").shape == (5, 16 * 24)
    assert project_feature_windows(windows, "sequence").shape == (5, 16, 24)


def test_sensor_wrench_rotates_to_base_and_shifts_moment():
    wrench = np.asarray([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    pose = np.asarray([0.0, 0.0, 0.0, 90.0, 0.0, 0.0])
    transformed = sensor_wrench_to_base(
        wrench,
        pose,
        tip_to_sensor_translation_m=(0.0, 1.0, 0.0),
    )
    assert np.allclose(transformed[:3], [0.0, 1.0, 0.0], atol=1.0e-8)
    assert np.allclose(transformed[3:], [0.0, 0.0, -1.0], atol=1.0e-8)


def test_contact_detector_hysteresis():
    detector = SchmittContactDetector(2.0, 1.0, 0.008, 0.020)
    assert detector.update(2.1, 0.000) is False
    assert detector.update(2.1, 0.009) is True
    assert detector.update(0.9, 0.010) is True
    assert detector.update(0.9, 0.031) is False


def test_error_gate_uses_force_vector_norm_maximum():
    target = np.zeros((2, 6))
    prediction = np.zeros((2, 6))
    prediction[1, :3] = [0.6, 0.8, 0.0]
    metrics = error_metrics(target, prediction)
    assert np.isclose(metrics["force_norm_max_n"], 1.0)


def test_episode_timing_rejects_a_hidden_long_gap():
    good = np.arange(2626, dtype=np.float64) / SAMPLE_HZ
    assert episode_timing_quality(good)["accepted"]
    with_gap = good.copy()
    with_gap[1000:] += 0.011
    assert not episode_timing_quality(with_gap)["accepted"]


def test_fast_recording_gate_requires_fresh_fast_status():
    assert _teleop_state_from_json('{"state":"FAST"}') == "FAST"
    assert _teleop_state_from_json("not-json") == ""
    assert _fresh_fast_state("FAST", 10.0, 10.4, 0.5)
    assert not _fresh_fast_state("SLOW", 10.0, 10.1, 0.5)
    assert not _fresh_fast_state("FAST", 10.0, 10.6, 0.5)
