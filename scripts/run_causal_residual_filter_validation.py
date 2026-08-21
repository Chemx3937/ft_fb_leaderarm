#!/usr/bin/env python3
"""Offline-only causal residual-filter and robust-gate diagnostic."""

import argparse
import hashlib
import json
import math
import sqlite3
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from geometry_msgs.msg import WrenchStamped
from rclpy.serialization import deserialize_message

import ft_fb_leaderarm.train_ablation as training
from ft_fb_leaderarm.contract import RateGate, SAMPLE_HZ, SchmittContactDetector
import run_physical_residual_validation as physical
import run_targeted6_validation_ablation as targeted


DEFAULT_OUTPUT = Path(
    "/home/vision/.ros/ft_fb_leaderarm/models/"
    "right_train13_causal_residual_filter_diagnostic_v3_20260821"
)
REFERENCE_REPORT = Path(
    "/home/vision/.ros/ft_fb_leaderarm/models/"
    "right_train13_physical_residual_diagnostic_v4_20260821/validation_report.json"
)
STATIONARY_BAGS = {
    "on_tare01_retry01": Path(
        "/home/vision/.ros/ft_fb_leaderarm/experiments/"
        "aft_stationary_cable_fixed_20260821/tare01_retry01/tare01_retry01_0.db3"
    ),
    "on_tare02": Path(
        "/home/vision/.ros/ft_fb_leaderarm/experiments/"
        "aft_stationary_cable_fixed_20260821/tare02/tare02_0.db3"
    ),
    "on_tare03": Path(
        "/home/vision/.ros/ft_fb_leaderarm/experiments/"
        "aft_stationary_cable_fixed_20260821/tare03/tare03_0.db3"
    ),
    "off_tare01": Path(
        "/home/vision/.ros/ft_fb_leaderarm/experiments/"
        "aft_stationary_controller_off_20260821/off_tare01/off_tare01_0.db3"
    ),
    "off_tare02": Path(
        "/home/vision/.ros/ft_fb_leaderarm/experiments/"
        "aft_stationary_controller_off_20260821/off_tare02/off_tare02_0.db3"
    ),
    "off_tare03": Path(
        "/home/vision/.ros/ft_fb_leaderarm/experiments/"
        "aft_stationary_controller_off_20260821/off_tare03/off_tare03_0.db3"
    ),
    "off_settle60_tare01": Path(
        "/home/vision/.ros/ft_fb_leaderarm/experiments/"
        "aft_stationary_controller_off_settle60_20260821/"
        "off_settle60_tare01/off_settle60_tare01_0.db3"
    ),
}
FILTERS = (
    "raw",
    "median3",
    "median5",
    "ema20hz",
    "ema10hz",
    "ema5hz",
    "ema2p5hz",
    "median3_ema20hz",
)
GATE_DEFINITIONS = {
    "strict_max_1n": "force norm max <= 1 N",
    "p95_1n": "force norm p95 <= 1 N",
    "p99_1n": "force norm p99 <= 1 N",
    "p95_1n_and_peak_2n": "p95 <= 1 N and max <= contact-on threshold 2 N",
    "p95_1n_and_no_false_contact": (
        "p95 <= 1 N and zero activation with existing 2 N / 8 ms detector"
    ),
    "p99_1n_and_peak_2n": "p99 <= 1 N and max <= contact-on threshold 2 N",
    "p99_1n_and_no_false_contact": (
        "p99 <= 1 N and zero activation with existing 2 N / 8 ms detector"
    ),
}


def causal_ema(values, cutoff_hz):
    values = np.asarray(values, dtype=np.float64)
    alpha = 1.0 - math.exp(-2.0 * math.pi * float(cutoff_hz) / SAMPLE_HZ)
    result = np.empty_like(values)
    result[0] = values[0]
    for index in range(1, len(values)):
        result[index] = result[index - 1] + alpha * (
            values[index] - result[index - 1]
        )
    return result


def causal_median(values, window):
    values = np.asarray(values, dtype=np.float64)
    padded = np.concatenate((np.repeat(values[:1], window - 1, axis=0), values))
    windows = np.lib.stride_tricks.sliding_window_view(padded, window, axis=0)
    return np.median(windows, axis=2)


def apply_filter(name, values):
    values = np.asarray(values, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 6 or not len(values):
        raise ValueError("wrench values must have shape [N,6] with N > 0")
    if name == "raw":
        return values.copy()
    if name == "median3":
        return causal_median(values, 3)
    if name == "median5":
        return causal_median(values, 5)
    if name == "ema20hz":
        return causal_ema(values, 20.0)
    if name == "ema10hz":
        return causal_ema(values, 10.0)
    if name == "ema5hz":
        return causal_ema(values, 5.0)
    if name == "ema2p5hz":
        return causal_ema(values, 2.5)
    if name == "median3_ema20hz":
        return causal_ema(causal_median(values, 3), 20.0)
    raise ValueError(f"unknown filter: {name}")


def longest_run(mask):
    padded = np.pad(np.asarray(mask, dtype=np.int8), (1, 1))
    edges = np.flatnonzero(np.diff(padded)).reshape(-1, 2)
    return int(np.max(edges[:, 1] - edges[:, 0])) if len(edges) else 0


def force_metrics(values):
    force_norm = np.linalg.norm(np.asarray(values)[:, :3], axis=1)
    detector = SchmittContactDetector(2.0, 1.2, 0.008, 0.020)
    activations = 0
    previous = False
    for index, value in enumerate(force_norm):
        contact = detector.update(value, index / SAMPLE_HZ)
        activations += int(contact and not previous)
        previous = contact
    over_1 = force_norm > 1.0
    at_least_2 = force_norm >= 2.0
    return {
        "samples": int(len(force_norm)),
        "force_norm_rmse_n": float(np.sqrt(np.mean(force_norm**2))),
        "force_norm_p95_n": float(np.percentile(force_norm, 95.0)),
        "force_norm_p99_n": float(np.percentile(force_norm, 99.0)),
        "force_norm_max_n": float(np.max(force_norm)),
        "over_1n_fraction": float(np.mean(over_1)),
        "over_1n_longest_samples": longest_run(over_1),
        "over_1n_longest_span_ms": max(0, longest_run(over_1) - 1)
        / SAMPLE_HZ
        * 1000.0,
        "at_least_2n_fraction": float(np.mean(at_least_2)),
        "at_least_2n_longest_samples": longest_run(at_least_2),
        "false_contact_activations": activations,
    }


def gate_pass(metrics, gate):
    if gate == "strict_max_1n":
        return metrics["force_norm_max_n"] <= 1.0
    if gate == "p95_1n":
        return metrics["force_norm_p95_n"] <= 1.0
    if gate == "p99_1n":
        return metrics["force_norm_p99_n"] <= 1.0
    if gate == "p95_1n_and_peak_2n":
        return metrics["force_norm_p95_n"] <= 1.0 and metrics["force_norm_max_n"] <= 2.0
    if gate == "p95_1n_and_no_false_contact":
        return metrics["force_norm_p95_n"] <= 1.0 and not metrics[
            "false_contact_activations"
        ]
    if gate == "p99_1n_and_peak_2n":
        return metrics["force_norm_p99_n"] <= 1.0 and metrics["force_norm_max_n"] <= 2.0
    if gate == "p99_1n_and_no_false_contact":
        return metrics["force_norm_p99_n"] <= 1.0 and not metrics[
            "false_contact_activations"
        ]
    raise ValueError(f"unknown gate: {gate}")


def evaluate_sessions(sessions):
    result = {}
    for filter_name in FILTERS:
        filtered = {
            name: apply_filter(filter_name, values) for name, values in sessions.items()
        }
        by_group = {name: force_metrics(values) for name, values in filtered.items()}
        aggregate = force_metrics(np.concatenate(list(filtered.values())))
        aggregate["over_1n_longest_samples"] = max(
            row["over_1n_longest_samples"] for row in by_group.values()
        )
        aggregate["over_1n_longest_span_ms"] = max(
            row["over_1n_longest_span_ms"] for row in by_group.values()
        )
        aggregate["at_least_2n_longest_samples"] = max(
            row["at_least_2n_longest_samples"] for row in by_group.values()
        )
        aggregate["false_contact_activations"] = sum(
            row["false_contact_activations"] for row in by_group.values()
        )
        gates = {
            gate: {
                "pass": gate_pass(aggregate, gate)
                and all(gate_pass(row, gate) for row in by_group.values()),
                "failed_groups": [
                    name for name, row in by_group.items() if not gate_pass(row, gate)
                ],
            }
            for gate in GATE_DEFINITIONS
        }
        result[filter_name] = {
            "aggregate": aggregate,
            "by_group": by_group,
            "gates_require_aggregate_and_every_group": gates,
        }
    return result


def load_stationary_wrench(path):
    path = Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    gate = RateGate(SAMPLE_HZ)
    values = []
    with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
        topic = connection.execute(
            "SELECT id, type FROM topics WHERE name = '/aft_sensor2/wrench'"
        ).fetchone()
        if topic is None or topic[1] != "geometry_msgs/msg/WrenchStamped":
            raise RuntimeError(f"{path}: missing expected AFT wrench topic")
        for stamp_ns, serialized in connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic[0],),
        ):
            if not gate.accept(stamp_ns * 1.0e-9):
                continue
            msg = deserialize_message(serialized, WrenchStamped)
            values.append(
                (
                    msg.wrench.force.x,
                    msg.wrench.force.y,
                    msg.wrench.force.z,
                    msg.wrench.torque.x,
                    msg.wrench.torque.y,
                    msg.wrench.torque.z,
                )
            )
    result = np.asarray(values, dtype=np.float64)
    if len(result) < 50.0 * SAMPLE_HZ or not np.isfinite(result).all():
        raise RuntimeError(f"{path}: stationary bag is incomplete or non-finite")
    return result


def step_response():
    values = np.zeros((300, 6), dtype=np.float64)
    step_index = 30
    values[step_index:, 0] = 4.0
    result = {}
    for name in FILTERS:
        filtered = apply_filter(name, values)
        force_norm = np.linalg.norm(filtered[:, :3], axis=1)
        crossing = np.flatnonzero(force_norm[step_index:] >= 2.0)
        detector = SchmittContactDetector(2.0, 1.2, 0.008, 0.020)
        activation = None
        for index, value in enumerate(force_norm):
            if detector.update(value, index / SAMPLE_HZ):
                activation = index
                break
        result[name] = {
            "assumed_step_n": 4.0,
            "threshold_n": 2.0,
            "threshold_crossing_delay_samples": int(crossing[0]),
            "threshold_crossing_delay_ms": float(crossing[0] / SAMPLE_HZ * 1000.0),
            "contact_on_delay_with_8ms_hold_ms": float(
                (activation - step_index) / SAMPLE_HZ * 1000.0
            ),
        }
    return result


def self_check():
    rng = np.random.default_rng(7)
    values = rng.normal(size=(40, 6))
    changed_future = values.copy()
    changed_future[25:] += 100.0
    for name in FILTERS:
        assert np.allclose(
            apply_filter(name, values)[:25], apply_filter(name, changed_future)[:25]
        ), f"{name} used future samples"
        assert np.allclose(apply_filter(name, values), apply_filter(name, values))
    assert longest_run([False, True, True, False, True]) == 2


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    self_check()
    if args.self_check:
        print("causal residual filter self-check: PASS")
        return

    output = Path(args.output_dir).expanduser().resolve()
    started = time.monotonic()
    training.project_feature_windows = targeted.diagnostic_projection
    training.ABLATIONS = {**training.ABLATIONS, **physical.CANDIDATES}
    splits = targeted.load_fixed_splits()
    if set(splits) != {"train", "validation"}:
        raise RuntimeError("offline diagnostic refuses any held-out test split")
    manifest = training.session_manifest(splits)
    deltas = physical.gravity_deltas(splits)
    payload = physical.identify_payload(splits["train"], deltas)
    residual_splits = physical.residualize(splits, deltas, payload)
    selected_name = "physical_residual_smoothed_dynamic_mlp"
    trained = training.train_candidate(
        selected_name, residual_splits, seed=8, **targeted.SETTINGS
    )
    model = {selected_name: (trained["model"], trained["mode"], trained["history"])}
    validation_residual = {}
    for session in residual_splits["validation"]:
        _, target, prediction = targeted.aligned_prediction(model, session)
        validation_residual[session.group] = target - prediction

    validation = evaluate_sessions(validation_residual)
    stationary = evaluate_sessions(
        {name: load_stationary_wrench(path) for name, path in STATIONARY_BAGS.items()}
    )
    reference = json.loads(REFERENCE_REPORT.read_text(encoding="utf-8"))
    reproduced_raw = validation["raw"]["aggregate"]
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scope": (
            "offline train13 retraining + protected validation3 + seven existing "
            "stationary bags; past held-out test not loaded"
        ),
        "approved": False,
        "runtime_or_acceptance_changed": False,
        "filter_state_contract": (
            "state reset for each session; EMA and median start from first sample"
        ),
        "stationary_rate_contract": (
            "read-only bag timestamps causally decimated from about 1000 Hz to 262.5 Hz"
        ),
        "sample_hz": SAMPLE_HZ,
        "model": selected_name,
        "training_settings": targeted.SETTINGS,
        "manifest": manifest,
        "physical_identification": payload,
        "stationary_bags": {name: str(path) for name, path in STATIONARY_BAGS.items()},
        "filters": list(FILTERS),
        "gate_definitions_candidates_only": GATE_DEFINITIONS,
        "gate_rule": "aggregate and every independent group must pass",
        "validation_residual": validation,
        "stationary_target_noise": stationary,
        "synthetic_step_response": step_response(),
        "unfiltered_reproduction": {
            "reference_report": str(REFERENCE_REPORT),
            "reference_max_n": reference["selected_validation"]["force_norm_max_n"],
            "reproduced_max_n": reproduced_raw["force_norm_max_n"],
            "absolute_max_difference_n": abs(
                reference["selected_validation"]["force_norm_max_n"]
                - reproduced_raw["force_norm_max_n"]
            ),
        },
        "elapsed_s": time.monotonic() - started,
    }
    output.mkdir(parents=True, exist_ok=False)
    (output / "split_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output / "filter_validation_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"[DONE] output={output}", flush=True)


if __name__ == "__main__":
    main()
