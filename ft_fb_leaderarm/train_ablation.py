#!/usr/bin/env python3
"""Train five contact-safe physical-FT ablations and seal only a <=1 N model."""

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import random
import time

import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset

from .contract import (
    ABLATIONS,
    BASE_FEATURE_DIM,
    SAMPLE_HZ,
    SAMPLE_PERIOD_S,
    SCHEMA_VERSION,
    error_metrics,
    project_feature_windows,
)
from .model import (
    RecurrentWrenchRegressor,
    WrenchRegressor,
    make_normalized_model,
    save_bundle,
)


@dataclass(frozen=True)
class Session:
    path: Path
    metadata: dict
    features: np.ndarray
    wrench: np.ndarray

    @property
    def group(self):
        return str(self.metadata["zero_set_id"])


def load_session(path):
    path = Path(path).expanduser().resolve()
    with np.load(path, allow_pickle=False) as archive:
        metadata = json.loads(str(archive["metadata"].item()))
        features = np.asarray(archive["features"], dtype=np.float32)
        wrench = np.asarray(archive["raw_wrench"], dtype=np.float32)
        stamps = np.asarray(archive["stamp_s"], dtype=np.float64)
    if int(metadata.get("schema_version", -1)) != SCHEMA_VERSION:
        raise RuntimeError(f"{path}: unsupported schema")
    if not bool(metadata.get("accepted", False)):
        raise RuntimeError(f"{path}: collector did not accept this episode")
    if not bool(metadata.get("free_space_only", False)):
        raise RuntimeError(f"{path}: episode is not marked free-space-only")
    if str(metadata.get("robot_side", "")) != "right":
        raise RuntimeError(f"{path}: only the right follower arm is supported")
    if abs(float(metadata.get("sample_hz", 0.0)) - SAMPLE_HZ) > 1.0e-9:
        raise RuntimeError(f"{path}: sample rate must be {SAMPLE_HZ} Hz")
    if not str(metadata.get("zero_set_id", "")).strip():
        raise RuntimeError(f"{path}: zero_set_id is missing")
    if not str(metadata.get("payload_id", "")).strip():
        raise RuntimeError(f"{path}: payload_id is missing")
    if not str(metadata.get("controller_config_hash", "")).strip():
        raise RuntimeError(f"{path}: controller_config_hash is missing")
    if (
        features.ndim != 2
        or features.shape[1] != BASE_FEATURE_DIM
        or wrench.shape != (len(features), 6)
        or stamps.shape != (len(features),)
        or len(features) < 32
        or not np.isfinite(features).all()
        or not np.isfinite(wrench).all()
        or not np.isfinite(stamps).all()
        or np.any(np.diff(stamps) <= 0.0)
    ):
        raise RuntimeError(f"{path}: array shape, finiteness, or time order is invalid")
    return Session(path, metadata, features, wrench)


def load_sessions(data_dir):
    paths = sorted(Path(data_dir).expanduser().resolve().glob("*.npz"))
    if not paths:
        raise RuntimeError(f"no .npz episodes found under {data_dir}")
    sessions = [load_session(path) for path in paths]
    contract_keys = (
        "ft_frame",
        "observer_input_frame",
        "payload_id",
        "controller_config_hash",
    )
    reference = sessions[0].metadata
    for session in sessions[1:]:
        for key in contract_keys:
            if session.metadata.get(key) != reference.get(key):
                raise RuntimeError(
                    f"{session.path}: {key} differs from the first episode"
                )
        if not np.allclose(
            session.metadata.get("zero_pose_deg", []),
            reference.get("zero_pose_deg", []),
            rtol=0.0,
            atol=1.0e-9,
        ):
            raise RuntimeError(f"{session.path}: zero_pose_deg contract differs")
    return sessions


def split_by_zero_set(sessions, seed):
    groups = sorted({session.group for session in sessions})
    if len(groups) < 3:
        raise RuntimeError(
            "at least three independent zero_set_id groups are required for "
            "train/validation/held-out test"
        )
    random.Random(seed).shuffle(groups)
    test_count = max(1, int(round(0.20 * len(groups))))
    validation_count = max(1, int(round(0.20 * len(groups))))
    while test_count + validation_count >= len(groups):
        if test_count >= validation_count and test_count > 1:
            test_count -= 1
        elif validation_count > 1:
            validation_count -= 1
        else:
            break
    test_groups = set(groups[:test_count])
    validation_groups = set(groups[test_count : test_count + validation_count])
    train_groups = set(groups) - test_groups - validation_groups
    splits = {
        "train": [row for row in sessions if row.group in train_groups],
        "validation": [
            row for row in sessions if row.group in validation_groups
        ],
        "test": [row for row in sessions if row.group in test_groups],
    }
    if any(not values for values in splits.values()):
        raise RuntimeError("group split produced an empty train/validation/test set")
    return splits


def build_matrix(sessions, mode, history, max_windows_per_session, seed):
    x_parts, y_parts = [], []
    for session_index, session in enumerate(sessions):
        end_indices = np.arange(history - 1, len(session.features))
        if len(end_indices) > max_windows_per_session:
            rng = np.random.default_rng(seed + session_index)
            end_indices = np.sort(
                rng.choice(
                    end_indices, size=max_windows_per_session, replace=False
                )
            )
        offsets = np.arange(history - 1, -1, -1)
        indices = end_indices[:, None] - offsets[None, :]
        windows = session.features[indices]
        x_parts.append(project_feature_windows(windows, mode))
        y_parts.append(session.wrench[end_indices])
    return (
        np.concatenate(x_parts, axis=0).astype(np.float32),
        np.concatenate(y_parts, axis=0).astype(np.float32),
    )


def predict_batches(model, x, batch_size=8192):
    outputs = []
    model.eval()
    with torch.inference_mode():
        for start in range(0, len(x), batch_size):
            outputs.append(
                model(torch.from_numpy(x[start : start + batch_size]))
                .cpu()
                .numpy()
            )
    return np.concatenate(outputs, axis=0)


def evaluate_all_windows(model, sessions, mode, history, batch_size=8192):
    """Evaluate every causal window without materializing a full history matrix."""
    targets, predictions = [], []
    offsets = np.arange(history - 1, -1, -1)
    for session in sessions:
        end_indices = np.arange(history - 1, len(session.features))
        for start in range(0, len(end_indices), batch_size):
            batch_ends = end_indices[start : start + batch_size]
            indices = batch_ends[:, None] - offsets[None, :]
            projected = project_feature_windows(
                session.features[indices], mode
            )
            targets.append(session.wrench[batch_ends])
            predictions.append(predict_batches(model, projected, batch_size))
    if not targets:
        raise RuntimeError("no causal windows were available for evaluation")
    target = np.concatenate(targets, axis=0)
    prediction = np.concatenate(predictions, axis=0)
    return error_metrics(target, prediction)


def train_candidate(
    name,
    splits,
    epochs,
    batch_size,
    learning_rate,
    max_windows_per_session,
    seed,
):
    mode, history, hidden_dims, architecture = ABLATIONS[name]
    matrices = {
        split: build_matrix(
            sessions,
            mode,
            history,
            max_windows_per_session,
            seed + index * 1000,
        )
        for index, (split, sessions) in enumerate(
            (("train", splits["train"]), ("validation", splits["validation"]))
        )
    }
    train_x, train_y = matrices["train"]
    validation_x, validation_y = matrices["validation"]
    feature_axes = tuple(range(train_x.ndim - 1))
    x_mean = train_x.mean(axis=feature_axes)
    x_std = np.maximum(train_x.std(axis=feature_axes), 1.0e-6)
    y_mean = train_y.mean(axis=0)
    y_std = np.maximum(train_y.std(axis=0), 1.0e-6)
    normalized_train_x = (train_x - x_mean) / x_std
    normalized_train_y = (train_y - y_mean) / y_std

    torch.manual_seed(seed)
    if architecture == "mlp":
        core = WrenchRegressor(train_x.shape[-1], hidden_dims)
    else:
        core = RecurrentWrenchRegressor(
            train_x.shape[-1], hidden_dims[0], architecture
        )
    optimizer = torch.optim.AdamW(
        core.parameters(), lr=learning_rate, weight_decay=1.0e-5
    )
    loader = DataLoader(
        TensorDataset(
            torch.from_numpy(normalized_train_x),
            torch.from_numpy(normalized_train_y),
        ),
        batch_size=batch_size,
        shuffle=True,
        generator=torch.Generator().manual_seed(seed),
    )
    best_state = None
    best_score = math.inf
    best_epoch = 0
    stale_epochs = 0
    patience = max(8, epochs // 6)
    for epoch in range(1, epochs + 1):
        core.train()
        for batch_x, batch_y in loader:
            optimizer.zero_grad(set_to_none=True)
            prediction = core(batch_x)
            error = prediction - batch_y
            force_norm_sq = torch.sum(error[:, :3] ** 2, dim=1)
            tail_count = max(1, int(math.ceil(0.05 * len(force_norm_sq))))
            loss = torch.mean(error**2) + 0.20 * torch.mean(
                torch.topk(force_norm_sq, tail_count).values
            )
            loss.backward()
            optimizer.step()
        normalized_model = make_normalized_model(
            core, x_mean, x_std, y_mean, y_std
        )
        validation_prediction = predict_batches(
            normalized_model, validation_x
        )
        metrics = error_metrics(validation_y, validation_prediction)
        score = metrics["force_norm_rmse_n"]
        if score + 1.0e-7 < best_score:
            best_score = score
            best_epoch = epoch
            best_state = {
                key: value.detach().cpu().clone()
                for key, value in core.state_dict().items()
            }
            stale_epochs = 0
        else:
            stale_epochs += 1
        if stale_epochs >= patience:
            break

    if best_state is None:
        raise RuntimeError(f"{name}: training produced no checkpoint")
    core.load_state_dict(best_state)
    model = make_normalized_model(core, x_mean, x_std, y_mean, y_std)
    validation_metrics = evaluate_all_windows(
        model, splits["validation"], mode, history
    )
    return {
        "name": name,
        "mode": mode,
        "history": history,
        "architecture": architecture,
        "model": model,
        "best_epoch": best_epoch,
        "train_samples": len(train_x),
        "validation_samples": validation_metrics["samples"],
        "validation": validation_metrics,
    }


def benchmark_runtime(model, session, mode, history, calls):
    available = len(session.features) - history + 1
    if available <= 0:
        raise RuntimeError("test episode is shorter than model history")
    end_indices = np.linspace(
        history - 1, len(session.features) - 1, min(calls, available), dtype=int
    )
    offsets = np.arange(history - 1, -1, -1)
    windows = session.features[end_indices[:, None] - offsets[None, :]]
    torch.set_num_threads(1)
    for window in windows[: min(100, len(windows))]:
        projected = project_feature_windows(window[None, :, :], mode)
        with torch.inference_mode():
            model(torch.from_numpy(projected))
    durations_ms = []
    for index in range(calls):
        window = windows[index % len(windows)]
        start_ns = time.perf_counter_ns()
        projected = project_feature_windows(window[None, :, :], mode)
        with torch.inference_mode():
            model(torch.from_numpy(projected))
        durations_ms.append((time.perf_counter_ns() - start_ns) * 1.0e-6)
    values = np.asarray(durations_ms, dtype=np.float64)
    return {
        "calls": calls,
        "mean_ms": float(values.mean()),
        "p99_ms": float(np.percentile(values, 99.0)),
        "max_ms": float(values.max()),
        "period_ms": SAMPLE_PERIOD_S * 1000.0,
    }


def session_manifest(splits):
    return {
        name: [
            {"path": str(row.path), "zero_set_id": row.group}
            for row in sessions
        ]
        for name, sessions in splits.items()
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--candidates",
        nargs="+",
        choices=tuple(ABLATIONS),
        default=list(ABLATIONS),
    )
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch-size", type=int, default=1024)
    parser.add_argument("--learning-rate", type=float, default=1.0e-3)
    parser.add_argument("--max-windows-per-session", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-force-error-n", type=float, default=1.0)
    parser.add_argument("--benchmark-calls", type=int, default=2000)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not (0.0 < args.max_force_error_n <= 1.0):
        raise SystemExit("--max-force-error-n must be within (0, 1.0]")
    if args.epochs < 1 or args.max_windows_per_session < 1:
        raise SystemExit("epochs and max-windows-per-session must be positive")
    if args.benchmark_calls < 2000:
        raise SystemExit("--benchmark-calls must be at least 2000 for approval")

    sessions = load_sessions(args.data_dir)
    splits = split_by_zero_set(sessions, args.seed)
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for index, name in enumerate(args.candidates):
        print(f"[ABLATION] training {name}", flush=True)
        result = train_candidate(
            name,
            splits,
            args.epochs,
            args.batch_size,
            args.learning_rate,
            args.max_windows_per_session,
            args.seed + index,
        )
        results.append(result)
        print(
            f"[ABLATION] {name}: validation max="
            f"{result['validation']['force_norm_max_n']:.4f} N, p95="
            f"{result['validation']['force_norm_p95_n']:.4f} N",
            flush=True,
        )

    selected = min(
        results,
        key=lambda row: (
            row["validation"]["force_norm_max_n"],
            row["validation"]["force_norm_p95_n"],
            row["validation"]["force_norm_rmse_n"],
        ),
    )
    test_metrics = evaluate_all_windows(
        selected["model"],
        splits["test"],
        selected["mode"],
        selected["history"],
    )
    benchmark = benchmark_runtime(
        selected["model"],
        splits["test"][0],
        selected["mode"],
        selected["history"],
        args.benchmark_calls,
    )
    accuracy_pass = (
        selected["validation"]["force_norm_max_n"] <= args.max_force_error_n
        and test_metrics["force_norm_max_n"] <= args.max_force_error_n
    )
    runtime_pass = (
        benchmark["p99_ms"] <= 0.80 * benchmark["period_ms"]
        and benchmark["max_ms"] <= benchmark["period_ms"]
    )
    approved = accuracy_pass and runtime_pass
    reference = sessions[0].metadata
    public_results = [
        {
            "name": row["name"],
            "feature_mode": row["mode"],
            "history": row["history"],
            "architecture": row["architecture"],
            "best_epoch": row["best_epoch"],
            "train_samples": row["train_samples"],
            "validation_samples": row["validation_samples"],
            "validation": row["validation"],
        }
        for row in results
    ]
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "approved": approved,
        "selection_uses": "validation_only",
        "selected_ablation": selected["name"],
        "accuracy_gate": {
            "maximum_force_vector_error_n": args.max_force_error_n,
            "validation_pass": (
                selected["validation"]["force_norm_max_n"]
                <= args.max_force_error_n
            ),
            "held_out_test_pass": (
                test_metrics["force_norm_max_n"] <= args.max_force_error_n
            ),
        },
        "runtime_gate": {
            "required_hz": SAMPLE_HZ,
            "pass": runtime_pass,
            "benchmark": benchmark,
        },
        "ablations": public_results,
        "held_out_test_selected_model_only": test_metrics,
        "sessions": session_manifest(splits),
    }
    (output_dir / "ablation_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
    )
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "approved": approved,
        "sample_hz": SAMPLE_HZ,
        "base_feature_dim": BASE_FEATURE_DIM,
        "base_feature_contract": "q_rad[6],dq_rad_s[6],causal_qdd_rad_s2[6]",
        "forbidden_runtime_features": [
            "measured_wrench",
            "measured_joint_torque",
            "raw_joint_torque",
            "task_error",
        ],
        "target_contract": "physical_ft_sensor_frame_[Fx,Fy,Fz,Mx,My,Mz]",
        "ablation": selected["name"],
        "feature_mode": selected["mode"],
        "history": selected["history"],
        "architecture": selected["architecture"],
        "zero_pose_deg": reference["zero_pose_deg"],
        "ft_frame": reference["ft_frame"],
        "observer_input_frame": reference["observer_input_frame"],
        "payload_id": reference["payload_id"],
        "controller_config_hash": reference["controller_config_hash"],
        "validation": selected["validation"],
        "held_out_test": test_metrics,
        "runtime_benchmark": benchmark,
        "max_force_error_gate_n": args.max_force_error_n,
        "ablation_report": "ablation_report.json",
    }
    save_bundle(selected["model"], metadata, output_dir)
    print(
        f"[RESULT] approved={approved} selected={selected['name']} "
        f"validation_max={selected['validation']['force_norm_max_n']:.4f} N "
        f"test_max={test_metrics['force_norm_max_n']:.4f} N "
        f"runtime_p99={benchmark['p99_ms']:.4f} ms",
        flush=True,
    )
    if not approved:
        print(
            "[RESULT] model.ts was retained for diagnosis but metadata marks it "
            "rejected; ft_contact_observer will refuse to load it.",
            flush=True,
        )
        raise SystemExit(2)


if __name__ == "__main__":
    main()
