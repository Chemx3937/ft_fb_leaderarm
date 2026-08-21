#!/usr/bin/env python3
"""Evaluate train-only pose/history KNN corrections on fixed validation3."""

import argparse
import hashlib
import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

import ft_fb_leaderarm.train_ablation as training
from ft_fb_leaderarm.contract import SAMPLE_HZ
import run_causal_residual_filter_validation as filtering
import run_physical_residual_validation as physical
import run_targeted6_validation_ablation as targeted


DEFAULT_OUTPUT = Path(
    "/home/vision/.ros/ft_fb_leaderarm/models/"
    "right_train13_pose_history_candidate_diagnostic_v2_20260821"
)
KINDS = ("posture", "dynamic", "slow_history")
SCALES = (0.25, 0.5, 1.0)
COMMON_HISTORY = 128
NEIGHBORS = 64


def window_mean(values, ends, width):
    prefix = np.vstack((np.zeros((1, values.shape[1])), np.cumsum(values, axis=0)))
    return (prefix[ends + 1] - prefix[ends + 1 - width]) / width


def compact_features(session, ends, kind):
    ends = np.asarray(ends, dtype=int)
    q = session.features[ends, :6]
    posture = np.concatenate((np.sin(q), np.cos(q)), axis=1)
    if kind == "posture":
        return posture.astype(np.float32)
    dynamic = np.concatenate(
        (
            posture,
            session.features[ends, 6:12],
            window_mean(session.features[:, 12:18], ends, 8),
        ),
        axis=1,
    )
    if kind == "dynamic":
        return dynamic.astype(np.float32)
    if kind == "slow_history":
        return np.concatenate(
            (
                dynamic,
                q - session.features[ends - 31, :6],
                q - session.features[ends - 127, :6],
                window_mean(session.features[:, 6:12], ends, 32),
                window_mean(session.features[:, 6:12], ends, 128),
            ),
            axis=1,
        ).astype(np.float32)
    raise ValueError(f"unknown feature kind: {kind}")


def sampled_ends(session, kind, session_index):
    first = COMMON_HISTORY - 1 if kind == "slow_history" else 7
    ends = np.arange(first, len(session.features))
    if len(ends) > 5000:
        ends = np.sort(
            np.random.default_rng(8 + session_index).choice(
                ends, size=5000, replace=False
            )
        )
    return ends


def build_correction_index(model, sessions, kind, model_history):
    features, corrections = [], []
    for session_index, session in enumerate(sessions):
        ends = sampled_ends(session, kind, session_index)
        baseline = targeted.predict_session(
            model, session, "smoothed_dynamic", model_history
        )
        features.append(compact_features(session, ends, kind))
        corrections.append(
            session.wrench[ends] - baseline[ends - (model_history - 1)]
        )
    train_x = np.concatenate(features).astype(np.float64)
    correction = np.concatenate(corrections).astype(np.float64)
    mean = train_x.mean(axis=0)
    std = np.maximum(train_x.std(axis=0), 1.0e-6)
    return cKDTree((train_x - mean) / std), correction, mean, std


def query_correction(index, values):
    tree, correction, mean, std = index
    _, neighbors = tree.query(
        (np.asarray(values, dtype=np.float64) - mean) / std,
        k=min(NEIGHBORS, len(correction)),
        workers=-1,
    )
    return np.mean(correction[neighbors], axis=1)


def aggregate_metrics(residuals, filter_name):
    filtered = {
        name: filtering.apply_filter(filter_name, values)
        for name, values in residuals.items()
    }
    by_group = {
        name: filtering.force_metrics(values) for name, values in filtered.items()
    }
    aggregate = filtering.force_metrics(np.concatenate(list(filtered.values())))
    aggregate["over_1n_longest_span_ms"] = max(
        row["over_1n_longest_span_ms"] for row in by_group.values()
    )
    aggregate["false_contact_activations"] = sum(
        row["false_contact_activations"] for row in by_group.values()
    )
    return {"aggregate": aggregate, "by_group": by_group}


def evaluate_candidates(model, correction_indices, sessions, model_history):
    residuals = {"baseline_common_window": {}}
    correction_norms = {kind: [] for kind in KINDS}
    for kind in KINDS:
        for scale in SCALES:
            residuals[f"baseline_plus_{kind}_knn64_x{scale:g}"] = {}
    for session in sessions:
        ends = np.arange(COMMON_HISTORY - 1, len(session.features))
        baseline = targeted.predict_session(
            model, session, "smoothed_dynamic", model_history
        )[ends - (model_history - 1)]
        target = session.wrench[ends]
        residuals["baseline_common_window"][session.group] = target - baseline
        for kind, index in correction_indices.items():
            correction = query_correction(
                index, compact_features(session, ends, kind)
            )
            correction_norms[kind].append(np.linalg.norm(correction[:, :3], axis=1))
            for scale in SCALES:
                residuals[f"baseline_plus_{kind}_knn64_x{scale:g}"][session.group] = (
                    target - baseline - scale * correction
                )
    result = {
        name: {
            "raw": aggregate_metrics(rows, "raw"),
            "ema5": aggregate_metrics(rows, "ema5hz"),
        }
        for name, rows in residuals.items()
    }
    return result, {
        kind: {
            "median_n": float(np.median(np.concatenate(values))),
            "p95_n": float(np.percentile(np.concatenate(values), 95.0)),
            "max_n": float(np.max(np.concatenate(values))),
        }
        for kind, values in correction_norms.items()
    }


def self_check():
    features = np.arange(200 * 18, dtype=np.float32).reshape(200, 18) / 1000.0
    session = training.Session(Path("probe"), {"zero_set_id": "probe"}, features, np.zeros((200, 6)))
    assert compact_features(session, [127], "posture").shape == (1, 12)
    assert compact_features(session, [127], "dynamic").shape == (1, 24)
    assert compact_features(session, [127], "slow_history").shape == (1, 48)
    tree = cKDTree([[0.0], [1.0]])
    correction = np.asarray([[2.0] * 6, [4.0] * 6])
    predicted = query_correction((tree, correction, np.zeros(1), np.ones(1)), [[0.5]])
    assert np.allclose(predicted, 3.0)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    self_check()
    if args.self_check:
        print("pose/history candidate self-check: PASS")
        return

    output = Path(args.output_dir).expanduser().resolve()
    started = time.monotonic()
    training.project_feature_windows = targeted.diagnostic_projection
    training.ABLATIONS = {**training.ABLATIONS, **physical.CANDIDATES}
    splits = targeted.load_fixed_splits()
    if set(splits) != {"train", "validation"}:
        raise RuntimeError("offline diagnostic refuses any held-out test split")
    deltas = physical.gravity_deltas(splits)
    payload = physical.identify_payload(splits["train"], deltas)
    residual_splits = physical.residualize(splits, deltas, payload)
    selected_name = "physical_residual_smoothed_dynamic_mlp"
    trained = training.train_candidate(
        selected_name, residual_splits, seed=8, **targeted.SETTINGS
    )
    indices = {
        kind: build_correction_index(
            trained["model"], residual_splits["train"], kind, trained["history"]
        )
        for kind in KINDS
    }
    candidates, correction_norms = evaluate_candidates(
        trained["model"], indices, residual_splits["validation"], trained["history"]
    )
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scope": "offline train13 fit + protected validation3 only; held-out not loaded",
        "approved": False,
        "runtime_or_acceptance_changed": False,
        "warning": "KNN corrections are diagnostic candidates, not runtime bundles",
        "base_model": selected_name,
        "common_history_samples": COMMON_HISTORY,
        "common_history_s": (COMMON_HISTORY - 1) / SAMPLE_HZ,
        "neighbors": NEIGHBORS,
        "correction_scales": list(SCALES),
        "train_sample_contract": "deterministic maximum 5000 windows per train session",
        "manifest": training.session_manifest(splits),
        "physical_identification": payload,
        "candidate_metrics": candidates,
        "validation_correction_force_norm": correction_norms,
        "elapsed_s": time.monotonic() - started,
    }
    output.mkdir(parents=True, exist_ok=False)
    (output / "split_manifest.json").write_text(
        json.dumps(report["manifest"], indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output / "pose_history_candidate_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"[DONE] output={output}", flush=True)


if __name__ == "__main__":
    main()
